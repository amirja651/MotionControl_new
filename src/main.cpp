// =============================
// File: main.cpp  (lean RT variant)
// Depends on: AccelStepper, TMCStepper, Preferences (Arduino-ESP32)
// =============================

#include <Arduino.h>
#include <Preferences.h>
#include <SimpleCLI.h>

#include "DirMultiplexer_lean.h"     // fast DIR mux
#include "MAE3Encoder_lean.h"        // MAE3 PWM encoder (lean)
#include "Pins_lean.h"               // pin maps
#include "PositionController.h"      // motion controller & RTOS task
#include "SystemDiagnostics_lean.h"  // diagnostics
#include "TMC5160Manager_lean.h"     // driver manager
#include "UnitConverter_lean.h"      // unit conversions
#include "VoltageMonitor_lean.h"     // voltage monitor

SimpleCLI cli;

Command cmdMotor;
Command cmdMove;
Command cmdControlMode;
Command cmdStop;
Command cmdEnable;
Command cmdDisable;
Command cmdCurrentPosition;
Command cmdLastPosition;
Command cmdSave;
Command cmdRestart;
Command cmdShow;
Command cmdHelp;
Command cmdTest;

using namespace pins_helpers;

// ---------- Project constants ----------
static constexpr uint8_t  kMotorCount         = 4;
static constexpr uint8_t  kLinearMotorIndex   = 0;      // M1 (index 0)
static constexpr uint8_t  kFirstRotaryIndex   = 1;      // M2..M4
static constexpr double   kDefaultLeadPitchUm = 200.0;  // default: 0.2 mm/rev = 200 µm
static constexpr uint32_t kSerialBaud         = 115200;
static constexpr uint32_t kWatchdogSeconds    = 15;

// ---------- Persistence (NVS/Preferences) ----------
Preferences gPrefs;  // namespace: "motion"

// ---------- Hardware blocks ----------
DirMultiplexer gDirMux(MultiplexerPins::S0, MultiplexerPins::S1, MultiplexerPins::DIR);

// Drivers (CS per channel)
TMC5160Manager gDriver[kMotorCount] = {TMC5160Manager(0, DriverPins::CS[0]),
                                       TMC5160Manager(1, DriverPins::CS[1]),
                                       TMC5160Manager(2, DriverPins::CS[2]),
                                       TMC5160Manager(3, DriverPins::CS[3])};

// Encoders (MAE3): one per channel; contract = one active at a time
// Pin, pullup=false, pulldown=false (external if you have), inverted=false
mae3::Mae3Encoder gEnc[kMotorCount] = {mae3::Mae3Encoder(0, {EncoderPins::SIGNAL[0], false, false, false}),
                                       mae3::Mae3Encoder(1, {EncoderPins::SIGNAL[1], false, false, false}),
                                       mae3::Mae3Encoder(2, {EncoderPins::SIGNAL[2], false, false, false}),
                                       mae3::Mae3Encoder(3, {EncoderPins::SIGNAL[3], false, false, false})};

// Position controllers
PositionController* gPC[kMotorCount] = {nullptr, nullptr, nullptr, nullptr};

// Voltage monitor (digital HIGH = OK)
VoltageMonitor gVmon(VoltageMonitorPins::POWER_3_3, MonitorMode::DIGITAL_, /*thr*/ 1, /*debounceMs*/ 5);

// Runtime state
volatile bool gVoltageDrop  = false;
uint8_t       gCurrentMotor = 0;

// Lead pitch (µm per revolution) — configurable at runtime
double gLeadPitchUm = kDefaultLeadPitchUm;

static const char* kKeyPitchUm = "sys.pitch_um";

// ---------- Forward declarations (place before setup()) ----------
static String             keyStable(uint8_t i);
static String             keyMode(uint8_t i);
static inline int         toInt(ControlMode m);
static inline ControlMode toMode(int v);

static inline bool isLinear(uint8_t idx);
static inline bool isRotary(uint8_t idx);

static void    applyUnitDefaults(uint8_t idx);
static void    seedCurrentFromEncoder(uint8_t idx);
static void    saveStableSteps(uint8_t idx, int32_t steps);
static int32_t loadStableSteps(uint8_t idx);

static void onVoltageDropISR();
static void onMovementComplete();
static void configureByDistance(PositionController& pc, int32_t current, int32_t target);

// CLI handlers
static void handleMotor(cmd* c);
static void handleMove(cmd* c);
static void handleControl(cmd* c);
static void handleStop(cmd* c);
static void handleEnable(cmd* c);
static void handleDisable(cmd* c);
static void handlePosition(cmd* c);
static void handleSaveOrigin(cmd* c);
static void handleLast(cmd* c);
static void handleConfPitch(cmd* c);

// CLI wiring + service
static void attachCli();
static void serviceCLI();
static void initializeCLI();

void setup()
{
    Serial.begin(kSerialBaud);
    delay(50);

    // NVS
    gPrefs.begin("motion", false /*rw*/);
    gLeadPitchUm = gPrefs.getDouble(kKeyPitchUm, kDefaultLeadPitchUm);

    // Diagnostics (optional pretty print)
    SystemDiagnostics::printSystemStatus();

    // GPIO safe init
    gDirMux.begin();

    // SPI drivers
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        gDriver[i].begin();
        gDriver[i].configureDriver_All_Motors(true /*StealthChop*/);
    }

    // Encoders
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        (void)gEnc[i].init();  // enable ISR; we keep them disabled logically per manager when not in use
    }

    // Position controllers
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        applyUnitDefaults(i);
        // With/without encoder pointer: we pass encoder for all; hybrid optional
        gPC[i] = new PositionController(i, gDriver[i], gDirMux, DriverPins::STEP[i], DriverPins::EN[i], gEnc[i]);
        gPC[i]->begin();
    }

    // RT task for all controllers
    initializePositionControllers();
    startPositionControlSystem();
    for (uint8_t i = 0; i < kMotorCount; ++i)
        gPC[i]->enable();

    // Movement-complete callback (persist target as stable)
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        gPC[i]->attachOnComplete(onMovementComplete);
    }

    // Voltage monitor
    gVmon.begin();
    gVmon.onDrop(onVoltageDropISR);

    // Load stable positions; ensure each motor is at its stable before new moves
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        const int32_t stable = loadStableSteps(i);
        gPC[i]->setCurrentPosition(stable);  // seed
        if (stable != 0)
        {
            // Optionally, cue a move to stable immediately
            gPC[i]->moveToSteps(stable, MovementType::SHORT_RANGE, ControlMode::OPEN_LOOP);
        }
    }

    // CLI
    attachCli();

    Serial.println("[READY]");
}

void loop()
{
    // Handle encoder absence-of-interrupt hooks (light)
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        gEnc[i].handleInAbsenceOfInterrupt();
        gPC[i]->handleMovementComplete();
    }

    // Voltage monitor debounce & edge handling
    gVmon.update();
    if (gVoltageDrop)
    {
        gVoltageDrop = false;
        // On drop: stop all moving motors and persist their *start* (already saved in handleMove)
        for (uint8_t i = 0; i < kMotorCount; ++i)
        {
            if (gPC[i]->isMoving())
            {
                gPC[i]->stop();
            }
        }
        Serial.println("[POWER] Drop detected → motors stopped; last start kept as stable");
    }

    // CLI
    serviceCLI();

    delay(1);
}

// Keys: "m<idx>.stable_steps", "m<idx>.mode", "sys.pitch_um"
static String keyStable(uint8_t i)
{
    return String("m") + String(i) + ".stable_steps";
}
static String keyMode(uint8_t i)
{
    return String("m") + String(i) + ".mode";
}

// Control modes persisted as int (OPEN_LOOP=0, HYBRID=1) for future-proofing
static inline int toInt(ControlMode m)
{
    return (m == ControlMode::HYBRID) ? 1 : 0;
}
static inline ControlMode toMode(int v)
{
    return v == 1 ? ControlMode::HYBRID : ControlMode::OPEN_LOOP;
}

// ---------- Helpers ----------
static inline bool isLinear(uint8_t idx)
{
    return idx == kLinearMotorIndex;
}
static inline bool isRotary(uint8_t idx)
{
    return idx >= kFirstRotaryIndex && idx < kMotorCount;
}

static void applyUnitDefaults(uint8_t idx)
{
    if (isLinear(idx))
    {
        UnitConverter::setDefaultMotorType(MotorType::LINEAR);
        UnitConverter::setDefaultMicrometers(gLeadPitchUm);
    }
    else
    {
        UnitConverter::setDefaultMotorType(MotorType::ROTATIONAL);
        // For rotary, micrometers not used; keep default resolution & microsteps
    }
}

// Read encoder once and set current step reference for the controller
static void seedCurrentFromEncoder(uint8_t idx)
{
    if (!gPC[idx])
        return;

    double pos_f = 0, ton = 0, toff = 0;
    // Try to read one fresh sample (non-blocking). If none, keep last.
    if (gEnc[idx].tryGetPosition(pos_f, ton, toff))
    {
        int32_t steps = 0;
        if (isLinear(idx))
        {
            // Encoder MAE3 gives 0..4095 → convert to steps using UnitConverter pulses→steps
            steps = UnitConverter::convertFromPulses(pos_f).TO_STEPS;
        }
        else
        {
            // Rotary: same conversion but interpreted as degrees later (we store in steps)
            steps = UnitConverter::convertFromPulses(pos_f).TO_STEPS;
        }
        gPC[idx]->setCurrentPosition(steps);
    }
}

// Save stable position in NVS
static void saveStableSteps(uint8_t idx, int32_t steps)
{
    gPrefs.putInt(keyStable(idx).c_str(), steps);
}

// Load stable position from NVS (default 0)
static int32_t loadStableSteps(uint8_t idx)
{
    return gPrefs.getInt(keyStable(idx).c_str(), 0);
}

static void onVoltageDropISR()
{
    gVoltageDrop = true;
}

static void onMovementComplete()
{
    // When a motor reaches target, persist the target as stable position
    if (!gPC[gCurrentMotor])
        return;
    const int32_t tgt = gPC[gCurrentMotor]->getTargetSteps();
    saveStableSteps(gCurrentMotor, tgt);
}

// Compute distance and configure speed/accel profile accordingly
static void configureByDistance(PositionController& pc, int32_t current, int32_t target)
{
    const int32_t d = abs(target - current);
    // Distance-type thresholds and speed tables are embedded in PositionController lean
    pc.configureSpeedForDistanceSteps(d);
}

// ---------- CLI handlers ----------
static void handleMotor(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    Argument numberArg = cmd.getArg("n");
    if (!numberArg.isSet())
        return;
    int n = numberArg.getValue().toInt();
    if (n < 1 || n > kMotorCount)
        return;
    gCurrentMotor = static_cast<uint8_t>(n - 1);
    applyUnitDefaults(gCurrentMotor);
    Serial.printf("[CLI] Current motor = %d\r\n", n);
}

static void handleMove(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // n (optional uses current), p (µm for linear, deg for rotary)
    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n] || !gPC[n]->isEnabled())
    {
        Serial.println("[ERR] Motor not enabled");
        return;
    }

    // Before any move, go to stable position (if not already there)
    const int32_t stable = loadStableSteps(n);
    const int32_t cur    = gPC[n]->getCurrentSteps();
    if (cur != stable)
    {
        configureByDistance(*gPC[n], cur, stable);
        gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);
        // NOTE: We return after queuing homing-to-stable; user can issue move again or wait.
        Serial.printf("[INFO] Homing to stable first: %ld steps\r\n", static_cast<long>(stable));
        return;
    }

    // Seed current from encoder as reference in the *current* revolution
    seedCurrentFromEncoder(n);

    const double p           = cmd.getArg("p").getValue().toDouble();
    int32_t      targetSteps = 0;

    if (isLinear(n))
    {
        // p in micrometers
        targetSteps = UnitConverter::convertFromMicrometers(p).TO_STEPS;
    }
    else
    {
        // p in degrees
        targetSteps = UnitConverter::convertFromDegrees(p).TO_STEPS;
    }

    const int32_t nowSteps = gPC[n]->getCurrentSteps();
    configureByDistance(*gPC[n], nowSteps, targetSteps);

    // Persist "start position" immediately — used if power drops mid-move
    saveStableSteps(n, nowSteps);

    // Open-loop move (per your logic). Hybrid path is available via CLI control command.
    if (gPC[n]->moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP))
    {
        Serial.printf("[MOVE] n=%d -> targetSteps=%ld (cur=%ld)\r\n",
                      n + 1,
                      static_cast<long>(targetSteps),
                      static_cast<long>(nowSteps));
    }
    else
    {
        Serial.println("[ERR] move command rejected");
    }
}

static void handleControl(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n])
        return;

    ControlMode m = ControlMode::OPEN_LOOP;
    if (cmd.getArg("h").isSet())
        m = ControlMode::HYBRID;
    gPC[n]->setControlMode(m);
    gPrefs.putInt(keyMode(n).c_str(), toInt(m));
    Serial.printf("[CTRL] motor %d mode=%s\r\n", n + 1, (m == ControlMode::HYBRID ? "HYBRID" : "OPEN_LOOP"));
}

static void handleStop(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n])
        return;
    gPC[n]->stop();
    // Stopped before target → keep last start (already saved) as stable
    Serial.printf("[STOP] motor %d\r\n", n + 1);
}

static void handleEnable(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n])
        return;
    gPC[n]->enable();
    Serial.printf("[EN] motor %d enabled\r\n", n + 1);
}

static void handleDisable(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n])
        return;
    gPC[n]->disable();
    Serial.printf("[EN] motor %d disabled\r\n", n + 1);
}

static void handlePosition(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n])
        return;

    const int32_t cur = gPC[n]->getCurrentSteps();
    const int32_t tgt = gPC[n]->getTargetSteps();
    if (isLinear(n))
    {
        const auto cvals = UnitConverter::convertFromSteps(cur);
        const auto tvals = UnitConverter::convertFromSteps(tgt);
        Serial.printf("[POS] M%d cur=%.3f µm tgt=%.3f µm steps=(%ld -> %ld)\r\n",
                      n + 1,
                      cvals.TO_MICROMETERS,
                      tvals.TO_MICROMETERS,
                      static_cast<long>(cur),
                      static_cast<long>(tgt));
    }
    else
    {
        const auto cvals = UnitConverter::convertFromSteps(cur);
        const auto tvals = UnitConverter::convertFromSteps(tgt);
        Serial.printf("[POS] M%d cur=%.3f deg tgt=%.3f deg steps=(%ld -> %ld)\r\n",
                      n + 1,
                      cvals.TO_DEGREES,
                      tvals.TO_DEGREES,
                      static_cast<long>(cur),
                      static_cast<long>(tgt));
    }
}

static void handleSaveOrigin(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // Save current as stable (origin or checkpoint)
    const int n = gCurrentMotor;
    if (!gPC[n])
        return;
    const int32_t cur = gPC[n]->getCurrentSteps();
    saveStableSteps(n, cur);
    Serial.printf("[SAVE] M%d stable_steps=%ld\r\n", n + 1, static_cast<long>(cur));
}

static void handleLast(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // Move to last stable
    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
            n = nv - 1;
    }
    if (!gPC[n] || !gPC[n]->isEnabled())
        return;
    const int32_t stable = loadStableSteps(n);
    const int32_t cur    = gPC[n]->getCurrentSteps();
    configureByDistance(*gPC[n], cur, stable);
    gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);
    Serial.printf("[LAST] M%d -> stable %ld\r\n", n + 1, static_cast<long>(stable));
}

static void handleConfPitch(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // conf pitch p=<micrometers_per_rev>  (affects linear motor only)
    const double p = cmd.getArg("p").getValue().toDouble();
    if (p > 0.0)
    {
        gLeadPitchUm = p;
        gPrefs.putDouble(kKeyPitchUm, gLeadPitchUm);
        if (isLinear(gCurrentMotor))
        {
            UnitConverter::setDefaultMicrometers(gLeadPitchUm);
        }
        Serial.printf("[CONF] pitch = %.3f µm/rev\r\n", gLeadPitchUm);
    }
}

// ---------- Setup & Loop ----------
static void attachCli()
{
    initializeCLI();

    cmdMotor.setCallback(handleMotor);
    cmdMove.setCallback(handleMove);
    cmdControlMode.setCallback(handleControl);
    cmdStop.setCallback(handleStop);
    cmdEnable.setCallback(handleEnable);
    cmdDisable.setCallback(handleDisable);
    cmdCurrentPosition.setCallback(handlePosition);
    cmdLastPosition.setCallback(handleLast);
    cmdSave.setCallback(handleSaveOrigin);

    // extra: configure linear pitch
    Command cmdConf = cli.addCmd("conf");
    cmdConf.addArg("pitch");  // subkey name (ignored)
    cmdConf.addArg("p", "200.0");
    cmdConf.setDescription("Configure linear pitch (µm/rev): conf pitch p=200.0");
    cmdConf.setCallback(handleConfPitch);
}

static void serviceCLI()
{
    // Non-blocking read
    while (Serial.available())
    {
        const String line = Serial.readStringUntil('\n');
        if (line.length() > 0)
        {
            cli.parse(line);
            if (cli.errored())
            {
                CommandError e = cli.getError();
                Serial.printf("[CLI ERR] %s %s\r\n", e.toString(), e.getCommand().toString());
            }
        }
    }
}

static void initializeCLI()
{
    cmdMotor = cli.addCmd("motor", handleMotor);
    cmdMotor.addArg("n", "1");  // motor number argument
    cmdMotor.setDescription("Select the motor");

    cmdMove = cli.addCmd("move", handleMove);
    cmdMove.addArg("n", "1");    // motor number argument
    cmdMove.addArg("p", "0.0");  // positional argument (um or deg)
    cmdMove.setDescription("Move the current motor to the target position");

    cmdControlMode = cli.addCmd("control", handleControl);
    cmdControlMode.addArg("n", "1");  // motor number argument
    cmdControlMode.addFlagArg("o");   // open loop
    cmdControlMode.addFlagArg("h");   // hybrid
    cmdControlMode.setDescription("Set control mode for current motor (open-loop or hybrid)");

    cmdStop = cli.addCmd("stop", handleStop);
    cmdStop.addArg("n", "1");  // motor number argument
    cmdStop.setDescription("Stop the current motor.");

    cmdEnable = cli.addCmd("enable", handleEnable);
    cmdEnable.addArg("n", "1");  // motor number argument
    cmdEnable.setDescription("Enable the current motor.");

    cmdDisable = cli.addCmd("disable", handleDisable);
    cmdDisable.addArg("n", "1");  // motor number argument
    cmdDisable.setDescription("Disable the current motor.");

    cmdCurrentPosition = cli.addCmd("position", handlePosition);
    cmdCurrentPosition.addArg("n", "1");  // motor number argument
    cmdCurrentPosition.setDescription("Show the current position of the current motor");

    cmdLastPosition = cli.addCmd("last", handleLast);
    cmdLastPosition.addArg("n", "1");  // motor number argument
    cmdLastPosition.setDescription("Show the last position of the current motor");

    cmdSave = cli.addCmd("save", handleSaveOrigin);
    cmdSave.addArg("n", "1");  // motor number argument
    cmdSave.addFlagArg("o");   // orgin
    cmdSave.setDescription("Save the current position as origin of current motor");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdShow = cli.addCmd("show");
    cmdShow.addArg("n", "1");  // motor number argument
    cmdShow.setDescription("Show the encoder and motor status");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");

    cmdTest = cli.addCmd("test");
    cmdTest.addArg("p", "0.0");  // positional argument (um or deg)
    cmdTest.setDescription("Test the conversion functions");
}

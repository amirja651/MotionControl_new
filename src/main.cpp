// =============================
// File: main.cpp  (lean RT variant)
// Depends on: AccelStepper, TMCStepper, Preferences (Arduino-ESP32)
// =============================

#include <Arduino.h>
#include <Preferences.h>
#include <SimpleCLI.h>

#include "CommandHistory.h"
#include "DirMultiplexer_lean.h"     // fast DIR mux
#include "MAE3Encoder_lean.h"        // MAE3 PWM encoder (lean)
#include "Pins_lean.h"               // pin maps
#include "PositionController.h"      // motion controller & RTOS task
#include "SystemDiagnostics_lean.h"  // diagnostics
#include "TMC5160Manager_lean.h"     // driver manager
#include "UnitConverter_lean.h"      // unit conversions
#include "VoltageMonitor_lean.h"     // voltage monitor

#pragma region "Define Variables"

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

//---------- Used in onHomingComplete ----
// Define globals for chaining
static bool    gPendingTargetAfterHoming = false;
static int32_t gPendingTargetSteps       = 0;

// ---------- History & Hotkeys ----------
CommandHistory<16> history;
const char*        kHotkeys[] = {"Q", "L", "K", "J"};
enum class CommandKey : uint8_t
{
    Q = 0,
    L = 1,
    K = 2,
    J = 3
};

// To display the time interval between K/Js
static uint32_t gLastKPressMs = 0;

// ---------- Project constants ----------
static constexpr uint8_t  kMotorCount         = 4;
static constexpr uint8_t  kLinearMotorIndex   = 0;        // M1 (index 0)
static constexpr uint8_t  kFirstRotaryIndex   = 1;        // M2..M4
static constexpr double   kDefaultLeadPitchUm = 200.0;    // default: 0.2 mm/rev = 200 µm
static constexpr int32_t  SPI_CLOCK           = 1000000;  // 1MHz SPI clock
static constexpr uint32_t kSerialBaud         = 115200;
static constexpr uint32_t kWatchdogSeconds    = 15;

// --- Rotary touch-window and speed profiles ---
static constexpr double kTouchWindowDeg = 1.0;    // final window near target in degrees
static constexpr float  kFastMaxSpeed   = 16000;  // steps/s for Rapid
static constexpr float  kFastAccel      = 24000;  // steps/s^2 for Rapid
static constexpr float  kTouchMaxSpeed  = 600;    // steps/s for Touch
static constexpr float  kTouchAccel     = 1200;   // steps/s^2 for Touch

// --- Chaining state (Rapid -> Touch) ---
static volatile bool    gPendingTouch    = false;
static volatile uint8_t gTouchMotorIdx   = 0;
static volatile int32_t gTouchFinalSteps = 0;

// ---------- Persistence (NVS/Preferences) ----------
Preferences gPrefs;  // namespace: "motion"

// ---------- Hardware blocks ----------
DirMultiplexer gDirMux(MultiplexerPins::S0, MultiplexerPins::S1, MultiplexerPins::DIR);

// Drivers (CS per channel)
TMC5160Manager gDriver[kMotorCount] = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

// Encoders (MAE3): one per channel; contract = one active at a time
// Pin, pullup=false, pulldown=false (external if you have), inverted=false
mae3::Mae3Encoder gEnc[kMotorCount] = {
    mae3::Mae3Encoder(0, {EncoderPins::SIGNAL[0], false, false, false}), mae3::Mae3Encoder(1, {EncoderPins::SIGNAL[1], false, false, false}), mae3::Mae3Encoder(2, {EncoderPins::SIGNAL[2], false, false, false}), mae3::Mae3Encoder(3, {EncoderPins::SIGNAL[3], false, false, false})};

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

#pragma endregion

#pragma region "Forward declarations"
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

// Helper Read EncoderPosition
static bool readEncoderPosition(uint8_t idx, double& pos_pulses);
static bool readEncoderPulses(uint8_t idx, double& pulses, double& ton_us, double& toff_us);

// Used for handleMove() method
static void onHomingComplete();

// --- Two-phase motion (Rapid -> Touch) declarations ---
static void        onRapidArrived();  // raw function pointer (no lambda)
static void        planTouch(uint8_t idx, int32_t finalSteps);
static inline void setCustomProfile(PositionController& pc, float vmax, float amax);

// Clear the current line and unhide the prompt
static void        clearLine(size_t nChars);
static void        redrawPrompt(const String& buf);
static inline void printPrompt();

#pragma endregion

void setup()
{
    SPI.begin(spi_sck(), spi_miso(), spi_mosi());
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(kSerialBaud);
    esp_log_level_set("*", ESP_LOG_INFO);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    // NVS
    gPrefs.begin("motion", false /*rw*/);
    gLeadPitchUm = gPrefs.getDouble(kKeyPitchUm, kDefaultLeadPitchUm);

    // Diagnostics (optional pretty print)
    SystemDiagnostics::printSystemStatus();

    const char* txt = "**NOT**";
#ifdef CONFIG_FREERTOS_CHECK_STACKOVERFLOW
    txt = "";
#endif
    log_d("Stack overflow checking %s enabled", txt);

    // GPIO safe init
    gDirMux.begin();

    // for disable all drivers pins - for avoid conflict in SPI bus
    // Initialize CS pins and turn them off with safety checks
    for (auto ch : {Chan::M0, Chan::M1, Chan::M2, Chan::M3})
    {
        pinMode(cs(ch), OUTPUT);
        digitalWrite(cs(ch), HIGH);
    }

    // SPI drivers
    log_d("TMC5160 init:");
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        // gDriver[i].begin();
        //  Initialize TMC5160 drivers
        if (!gDriver[i].begin())
        {
            printf("--- SPI/driver[%d] init ❌ failed\r\n", i + 1);
        }
        else
        {
            gDriver[i].configureDriver_All_Motors(true);  // true = StealthChop
            // gDriver[i].setRmsCurrent(350);                 // mA
            // gDriver[i].setIrun(16);
            // gDriver[i].setIhold(8);
            // gDriver[i].setMicrosteps(32);

            // Optional: quick sanity check
            if (!gDriver[i].testConnection(true))
            {
                printf("--- SPI/driver[%d] ⚠️ not responding\r\n", i + 1);
            }
            else
            {
                printf("--- SPI/driver[%d] is 👍 OK\r\n", i + 1);
            }

            // Set a reasonable value for the rotary (start experimentally; the smaller → the faster the SpreadCycle):
            const bool isRot = (i != kLinearMotorIndex);
            if (isRot)
            {
                gDriver[i].setTPWMTHRS(400);  // switch to SpreadCycle at higher speed
            }
            else
            {
                gDriver[i].setTPWMTHRS(0xFFFF);  // keep StealthChop for linear quiet moves
            }
        }
    }

    // Encoders
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        (void)gEnc[i].init();  // enable ISR; we keep them disabled logically per manager when not in use
    }

    // Position controllers
    log_d("Position Controller init:");
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        applyUnitDefaults(i);
        // With/without encoder pointer: we pass encoder for all; hybrid optional
        gPC[i] = new PositionController(i, gDriver[i], gDirMux, DriverPins::STEP[i], DriverPins::EN[i], gEnc[i]);
        if (!gPC[i]->begin())
        {
            printf("--- Motor[%d] init ❌ failed\r\n", i + 1);
        }
        else
        {
            printf("--- Motor[%d] initialized\r\n", i + 1);
        }
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
    // Initialize quick monitor
    if (gVmon.begin())
    {
        gVmon.setHysteresis(50);  // Optional: ±50 window
        gVmon.onDrop(onVoltageDropISR);
        // log_d("Voltage monitor initialized!");
    }

#if false
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
#endif

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

// Read encoder once and set current step reference for the controller
static void seedCurrentFromEncoder_2(uint8_t idx)
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

// Read encoder once and set current step reference for the controller
static void seedCurrentFromEncoder(uint8_t idx)
{
    if (!gPC[idx])
        return;

    const int32_t curSteps  = gPC[idx]->getCurrentSteps();
    int32_t       seedSteps = curSteps;
    const int32_t ms        = UnitConverter::getDefaultMicrosteps();
    int32_t       turns     = isLinear(idx) ? (curSteps / ms) : 0;  // rotary → force 0 turns

    // Optional: enable only the selected encoder to reduce ISR load
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        if (i == idx)
            gEnc[i].enable();
        else
            gEnc[i].disable();
    }

    double pos_f = 0.0, ton = 0.0, toff = 0.0;
    if (gEnc[idx].tryGetPosition(pos_f, ton, toff))
    {
        // Convert encoder pulses (0..4095) to steps within 1 revolution
        const int32_t withinSteps = UnitConverter::convertFromPulses(pos_f).TO_STEPS;

        // Rebuild absolute seed:
        //  - linear:  (turns * ms) + withinSteps
        //  - rotary:  (0 * ms)     + withinSteps
        seedSteps = (turns * ms) + withinSteps;

        // Set precise current position before any movement
        gPC[idx]->setCurrentPosition(seedSteps);
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

#pragma region "CLI handlers"
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

static void handleMove_2(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // n (optional uses current), p (µm for linear, deg for rotary)
    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
        {
            n             = nv - 1;
            gCurrentMotor = static_cast<uint8_t>(n);
            applyUnitDefaults(gCurrentMotor);
        }
    }
    if (!gPC[n] || !gPC[n]->isEnabled())
    {
        Serial.println("[ERR] Motor not enabled");
        return;
    }

    //-----------------------------------------------------------------

    //-----------------------------------------------------------------
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

    //-----------------------------------------------------------------
    // Before any move, go to stable position (if not already there)
    int32_t stable = loadStableSteps(n);  // amir
    Serial.printf("[INFO] Stable: (%ld steps)\r\n", static_cast<long>(stable));

    if (isRotary(n))
    {
        // Convert steps → degrees, clamp to [0.05°, 359.95°], then back to steps
        double deg = UnitConverter::convertFromSteps(stable).TO_DEGREES;

#if false        
            // --- Clamp stable range for rotary motors ---
            if (deg < 0.05)
                deg = 0.05;
            if (deg > 359.95)
                deg = 359.95;


        // wrap-around behavior (e.g., 370° → 10° instead of 359.95°):
        deg = fmod(deg, 360.0);
        if (deg < 0.0)
            deg += 360.0;
#endif
        stable = UnitConverter::convertFromDegrees(deg).TO_STEPS;
        Serial.printf("[INFO] Stable with wrap-around: (%ld steps)\r\n", static_cast<long>(stable));
    }

    //-----------------------------------------------------------------
    // Seed current from encoder as reference in the *current* revolution
    seedCurrentFromEncoder(n);
    const int32_t curSteps = gPC[n]->getCurrentSteps();

    // In handleMove(cmd) after computing `targetSteps`:
    const int32_t nowSteps = gPC[n]->getCurrentSteps();

    // Rotary two-phase logic
    if (isRotary(n))
    {
        const auto curCV = UnitConverter::convertFromSteps(nowSteps);
        const auto tgtCV = UnitConverter::convertFromSteps(targetSteps);
        double     ddeg  = fabs(tgtCV.TO_DEGREES - curCV.TO_DEGREES);

        if (ddeg > kTouchWindowDeg)
        {
            // Phase 1: Rapid to pre-target (target - sign*window)
            const double  dir      = (tgtCV.TO_DEGREES >= curCV.TO_DEGREES) ? +1.0 : -1.0;
            const double  preD     = tgtCV.TO_DEGREES - dir * kTouchWindowDeg;
            const int32_t preSteps = UnitConverter::convertFromDegrees(preD).TO_STEPS;

            // Save start point in case of power drop
            saveStableSteps(n, nowSteps);

            // Rapid profile
            setCustomProfile(*gPC[n], kFastMaxSpeed, kFastAccel);
            gPC[n]->moveToSteps(preSteps, MovementType::LONG_RANGE, ControlMode::OPEN_LOOP);

            // Chain Phase 2 (Touch) after homing-to-pre target:
            // Use a static callback (no lambdas) because attachOnComplete takes void(*)()
            static bool    s_pendingTouch = false;
            static uint8_t s_motorIdx     = 0;
            static int32_t s_finalTarget  = 0;

            s_pendingTouch = true;
            s_motorIdx     = n;
            s_finalTarget  = targetSteps;

            gPC[n]->attachOnComplete(
                []()
                {
                    // NOTE: if your attachOnComplete only accepts function pointers, replace this
                    // lambda with a static function and use globals: see previous message for pattern.
                });

            // If your PositionController requires a raw function pointer:
            extern void onRapidArrived();  // forward-declare a static function
            gPC[n]->attachOnComplete(onRapidArrived);

            Serial.printf("[MOVE] Rapid to pre-target (deg window=%.2f)\r\n", kTouchWindowDeg);
            return;
        }
        // Else small move → do single-phase touch:
        setCustomProfile(*gPC[n], kTouchMaxSpeed, kTouchAccel);
    }

    // Linear: normal distance-based config
    configureByDistance(*gPC[n], nowSteps, targetSteps);
    saveStableSteps(n, nowSteps);
    gPC[n]->moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);

    if (curSteps != stable)
    {
        configureByDistance(*gPC[n], curSteps, stable);

        // 1. Move to stable
        gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);

        // 2. When the motor reaches 'stable', automatically move to the requested target
        // Prepare next target for after homing
        gPendingTargetAfterHoming = true;
        gPendingTargetSteps       = targetSteps;
        // gPC[n]->attachOnComplete(onHomingComplete);

        // NOTE: We return after queuing homing-to-stable; user can issue move again or wait.
        Serial.printf("\r\n[INFO] Homing to stable first: stable (%ld steps), current (%ld steps)\r\n\r\n", static_cast<long>(stable), static_cast<long>(curSteps));
        return;
    }

    //-----------------------------------------------------------------
    // Seed current from encoder as reference in the *current* revolution
    seedCurrentFromEncoder(n);

    const int32_t nowSteps2 = gPC[n]->getCurrentSteps();
    configureByDistance(*gPC[n], nowSteps, targetSteps);

    // Persist "start position" immediately — used if power drops mid-move
    saveStableSteps(n, nowSteps2);

    // Open-loop move (per your logic). Hybrid path is available via CLI control command.
    if (gPC[n]->moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP))
    {
        Serial.printf("\r\n[MOVE] n=%d -> targetSteps=%ld (cur=%ld)\r\n", n + 1, static_cast<long>(targetSteps), static_cast<long>(nowSteps2));
    }
    else
    {
        Serial.println("[ERR] move command rejected");
    }
}

static void handleMove(cmd* c)
{
    Command  cmd(c);
    Argument aN = cmd.getArg("n");
    Argument aP = cmd.getArg("p");

    // ---- Resolve motor index (default: current selection) ----
    uint8_t n = gCurrentMotor;
    if (aN.isSet())
    {
        const int nv = aN.getValue().toInt();
        if (nv < 1 || nv > kMotorCount)
        {
            Serial.println("[ERR] motor id out of range (1..4)");
            return;
        }
        n             = static_cast<uint8_t>(nv - 1);
        gCurrentMotor = static_cast<uint8_t>(n);
    }
    if (!gPC[n])
    {
        Serial.println("[ERR] motor controller not initialized");
        return;
    }
    if (!gPC[n]->isEnabled())
    {
        Serial.println("[ERR] motor is disabled");
        return;
    }

    // ---- Make unit defaults consistent with motor type ----
    applyUnitDefaults(n);

    // ---- Load stable; for rotary clamp to [0.05°, 359.95°] ----
    int32_t stable = loadStableSteps(n);
    if (isRotary(n))
    {
        auto   cv  = UnitConverter::convertFromSteps(stable);
        double deg = cv.TO_DEGREES;
        if (deg < 0.05)
            deg = 0.05;
        if (deg > 359.95)
            deg = 359.95;
        stable = UnitConverter::convertFromDegrees(deg).TO_STEPS;
    }
    Serial.printf("[INFO] Stable: (%ld steps)\r\n", static_cast<long>(stable));

    // ---- Seed current position: turns from stepper, within-turn from encoder ----
    const int32_t ms          = UnitConverter::getDefaultMicrosteps();
    int32_t       curStepsRaw = gPC[n]->getCurrentSteps();
    int32_t       seedSteps   = curStepsRaw;

    // For linear: keep turns from stepper; for rotary: force turns = 0
    int32_t turns = isLinear(n) ? (curStepsRaw / ms) : 0;

    // Enable only the selected encoder to reduce ISR load, then try to read it
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        if (i == n)
            gEnc[i].enable();
        else
            gEnc[i].disable();
    }
    double pos_f = 0.0, ton = 0.0, toff = 0.0;
    if (gEnc[n].tryGetPosition(pos_f, ton, toff))
    {
        const int32_t withinSteps = UnitConverter::convertFromPulses(pos_f).TO_STEPS;
        seedSteps                 = (turns * ms) + withinSteps;  // rotary: turns==0 → withinSteps
        gPC[n]->setCurrentPosition(seedSteps);                   // align stepper to encoder within-turn
        Serial.printf("pos_f: %.2f\r\n", pos_f);
    }

    // ---- If not at stable, go to stable first (single move) ----
    if (seedSteps != stable)
    {
        configureByDistance(*gPC[n], seedSteps, stable);
        if (gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP))
        {
            Serial.printf("[INFO] Homing to stable first: stable (%ld steps), current (%ld steps)\r\n", static_cast<long>(stable), static_cast<long>(seedSteps));
        }
        else
        {
            Serial.println("[ERR] homing-to-stable rejected");
        }
        return;  // user may issue the move again after homing completes
    }

    // ---- Parse target argument ----
    if (!aP.isSet())
    {
        Serial.println("[ERR] missing argument p=<position> (µm for linear, deg for rotary)");
        return;
    }
    const double p = aP.getValue().toDouble();

    // ---- Compute target steps based on motor type ----
    int32_t targetSteps = 0;
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

    // ---- Rotary two-phase motion: Rapid -> Touch (window) ----
    const int32_t nowSteps = gPC[n]->getCurrentSteps();  // after seeding: equals 'stable'
    if (isRotary(n))
    {
        const auto   curCV = UnitConverter::convertFromSteps(nowSteps);
        const auto   tgtCV = UnitConverter::convertFromSteps(targetSteps);
        const double ddeg  = fabs(tgtCV.TO_DEGREES - curCV.TO_DEGREES);

        if (ddeg > kTouchWindowDeg)
        {
            // Phase 1: Rapid to pre-target (target -/+ window)
            const double  dir      = (tgtCV.TO_DEGREES >= curCV.TO_DEGREES) ? +1.0 : -1.0;
            const double  preDeg   = tgtCV.TO_DEGREES - dir * kTouchWindowDeg;
            const int32_t preSteps = UnitConverter::convertFromDegrees(preDeg).TO_STEPS;

            // Persist start in case of power drop
            saveStableSteps(n, nowSteps);

            // Fast profile for Rapid
            setCustomProfile(*gPC[n], kFastMaxSpeed, kFastAccel);

            // Queue Rapid move and chain Touch via onRapidArrived()
            if (gPC[n]->moveToSteps(preSteps, MovementType::LONG_RANGE, ControlMode::OPEN_LOOP))
            {
                gPendingTouch    = true;
                gTouchMotorIdx   = n;
                gTouchFinalSteps = targetSteps;

                // IMPORTANT: set Rapid-arrival callback (raw function pointer)
                gPC[n]->attachOnComplete(onRapidArrived);

                Serial.printf("[MOVE] Rapid to pre-target (deg window=%.2f)\r\n", kTouchWindowDeg);
            }
            else
            {
                Serial.println("[ERR] Rapid move rejected");
            }
            return;  // Touch will be scheduled in onRapidArrived()
        }

        // Small distance: single-phase Touch only
        setCustomProfile(*gPC[n], kTouchMaxSpeed, kTouchAccel);
    }

    // ---- Linear (or small rotary): single-phase with distance-based profile ----
    configureByDistance(*gPC[n], nowSteps, targetSteps);

    // Persist start in case of power drop (store the start point)
    saveStableSteps(n, nowSteps);

    // Queue final move
    if (gPC[n]->moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP))
    {
        if (isLinear(n))
        {
            Serial.printf("[MOVE] linear: target=%.3f µm (steps %ld)\r\n", p, static_cast<long>(targetSteps));
        }
        else
        {
            Serial.printf("[MOVE] rotary: target=%.3f° (steps %ld)\r\n", p, static_cast<long>(targetSteps));
        }
    }
    else
    {
        Serial.println("[ERR] move command rejected");
    }
}

static void onHomingComplete()
{
    if (gPendingTargetAfterHoming && gPC[gCurrentMotor])
    {
        gPendingTargetAfterHoming = false;

        // Seed current from encoder as reference in the *current* revolution
        seedCurrentFromEncoder(gCurrentMotor);

        const int32_t nowSteps = gPC[gCurrentMotor]->getCurrentSteps();
        configureByDistance(*gPC[gCurrentMotor], nowSteps, gPendingTargetSteps);

        // Persist "start position" immediately — used if power drops mid-move
        saveStableSteps(gCurrentMotor, nowSteps);

        // Open-loop move (per your logic). Hybrid path is available via CLI control command.
        if (gPC[gCurrentMotor]->moveToSteps(gPendingTargetSteps, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP))
        {
            Serial.printf("[MOVE] n=%d -> targetSteps=%ld (cur=%ld)\r\n", gCurrentMotor + 1, static_cast<long>(gPendingTargetSteps), static_cast<long>(nowSteps));
        }
        else
        {
            Serial.println("[ERR] move command rejected");
        }

        Serial.printf("[INFO] After homing, moving to new target (%ld steps)\r\n", (long)gPendingTargetSteps);
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
        Serial.printf("[POS] M%d cur=%.3f µm tgt=%.3f µm steps=(%ld -> %ld)\r\n", n + 1, cvals.TO_MICROMETERS, tvals.TO_MICROMETERS, static_cast<long>(cur), static_cast<long>(tgt));
    }
    else
    {
        const auto cvals = UnitConverter::convertFromSteps(cur);
        const auto tvals = UnitConverter::convertFromSteps(tgt);
        Serial.printf("[POS] M%d cur=%.3f deg tgt=%.3f deg steps=(%ld -> %ld)\r\n", n + 1, cvals.TO_DEGREES, tvals.TO_DEGREES, static_cast<long>(cur), static_cast<long>(tgt));
    }
}

static void handleSaveOrigin(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // Save current as stable (origin or checkpoint)
    const int n = gCurrentMotor;
    if (!gPC[n])
        return;

    // Seed current from encoder as reference in the *current* revolution
    seedCurrentFromEncoder(n);

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

#pragma endregion

#pragma region "CLI"
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

static void serviceCLI_2()
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

// --- ESC processing for ↑/↓ ---
static void serviceCLI()
{
    static String  inputBuffer;
    static uint8_t escState = 0;  // 0: normal, 1: got ESC, 2: got '['

    while (Serial.available())
    {
        char c = Serial.read();

        // --- ESC processing for ↑/↓ ---
        if (escState == 0 && c == '\x1b')
        {
            escState = 1;
            continue;
        }
        if (escState == 1)
        {
            if (c == '[')
            {
                escState = 2;
                continue;
            }
            escState = 0;
        }
        if (escState == 2)
        {
            if (c == 'A')
            {  // Up
                inputBuffer = history.up();
                redrawPrompt(inputBuffer);
            }
            else if (c == 'B')
            {  // Down
                inputBuffer = history.down();
                redrawPrompt(inputBuffer);
            }
            escState = 0;
            continue;
        }

        // --- Enter ---
        if (c == '\r' || c == '\n')
        {
            if (inputBuffer.length() > 0)
            {
                Serial.print("\r\n# ");
                Serial.println(inputBuffer);
                cli.parse(inputBuffer);  // SimpleCLI
                history.push(inputBuffer);
                history.resetCursor();
                inputBuffer = "";
            }
            else
            {
                Serial.println();
            }
            printPrompt();
            continue;
        }

        // --- Backspace ---
        if (c == '\b' || c == 127)
        {
            if (inputBuffer.length() > 0)
            {
                inputBuffer.remove(inputBuffer.length() - 1);
                Serial.print("\b \b");
            }
            continue;
        }

        // --- Hotkeys: Q / L / K / J ---
        const char upper = (c >= 'a' && c <= 'z') ? (c - 32) : c;

        // Q: Go to the last stable position
        if (upper == kHotkeys[(int)CommandKey::Q][0])
        {
            const uint8_t n = gCurrentMotor;
            if (gPC[n] && !gPC[n]->isMoving())
            {
                const int32_t stable = loadStableSteps(n);
                const int32_t cur    = gPC[n]->getCurrentSteps();
                configureByDistance(*gPC[n], cur, stable);
                gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);
                Serial.printf("\r\n[LAST] M%d -> stable %ld\r\n", n + 1, (long)stable);
                printPrompt();
            }
            continue;
        }

        // L: Display position status
        if (upper == kHotkeys[(int)CommandKey::L][0])
        {
            const int n = gCurrentMotor;
            if (gPC[n])
            {
                const int32_t cur = gPC[n]->getCurrentSteps();
                const int32_t tgt = gPC[n]->getTargetSteps();
                if (isLinear(n))
                {
                    const auto cvals = UnitConverter::convertFromSteps(cur);
                    const auto tvals = UnitConverter::convertFromSteps(tgt);
                    Serial.printf("\r\n[POS] M%d cur=%.3f µm  tgt=%.3f µm  (steps %ld -> %ld)\r\n", n + 1, cvals.TO_MICROMETERS, tvals.TO_MICROMETERS, (long)cur, (long)tgt);
                }
                else
                {
                    const auto cvals = UnitConverter::convertFromSteps(cur);
                    const auto tvals = UnitConverter::convertFromSteps(tgt);
                    Serial.printf("\r\n[POS] M%d cur=%.3f°  tgt=%.3f°  (steps %ld -> %ld)\r\n", n + 1, cvals.TO_DEGREES, tvals.TO_DEGREES, (long)cur, (long)tgt);
                }
                printPrompt();
                Serial.print(inputBuffer);
            }
            continue;
        }

        // K: Show encoder status + time interval between presses
        if (upper == kHotkeys[(int)CommandKey::K][0])
        {
            const uint32_t now = millis();
            const uint32_t dt  = (gLastKPressMs == 0) ? 0 : (now - gLastKPressMs);
            gLastKPressMs      = now;

            const int n = gCurrentMotor;

            double     pos_f = 0.0, ton = 0.0, toff = 0.0;
            const bool enabled = readEncoderPulses(n, pos_f, ton, toff);

            Serial.printf("\r\n[ENC] M%d enabled=%s", n + 1, enabled ? "YES" : "NO");
            if (enabled)
            {
                const float encoderAngle = UnitConverter::convertFromPulses(pos_f).TO_DEGREES;
                Serial.print(F("  Encoder Position: "));
                Serial.print(encoderAngle);
                Serial.print(F("° ("));
                Serial.print(pos_f, 0);
                Serial.println(F(" pulses)"));
            }
            else
            {
                log_w("Encoder not enabled or no new data");
            }

            Serial.printf("  dt=%u ms\r\n", dt);
            printPrompt();
            Serial.print(inputBuffer);
            continue;
        }

        // J
        if (upper == kHotkeys[(int)CommandKey::J][0])
        {
            continue;
        }

        // --- Normal character: append to buffer and echo ---
        inputBuffer += c;
        Serial.print(c);
    }

    // CLI error report (if parse was invalid)
    if (cli.errored())
    {
        CommandError e = cli.getError();
        Serial.printf("[CLI ERR] %s %s\r\n", e.toString().c_str(), e.getCommand());
        printPrompt();
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

#pragma endregion

// Read the current position from the encoder (degrees or micrometers)
static bool readEncoderPosition(uint8_t idx, double& pos_pulses)
{
    double ton = 0, toff = 0;
    if (!(gEnc[idx].enable() == mae3::Status::Ok))
        gEnc[idx].enable();
    if (gEnc[idx].tryGetPosition(pos_pulses, ton, toff))
    {
        return true;
    }
    return false;
}

static bool readEncoderPulses(uint8_t idx, double& pulses, double& ton_us, double& toff_us)
{
    // Only one encoder is active (optional but recommended)
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        if (i == idx)
            gEnc[i].enable();
        else
            gEnc[i].disable();
    }
    return gEnc[idx].tryGetPosition(pulses, ton_us, toff_us);
}

#pragma region "Helpers"
// ---------- Helpers ----------
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
    // Typical defaults (adjust if yours differ)
    UnitConverter::setDefaultResolution(4096);      // pulses per rev
    UnitConverter::setDefaultMicrometers(200.0);    // µm per rev (0.2 mm lead)
    UnitConverter::setDefaultMicrosteps(200 * 64);  // 200 steps * 64 µsteps = 12800

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

// Clear the current line and unhide the prompt
static void clearLine(size_t nChars)
{
    Serial.print("\r");  // Return to the beginning of the line
    for (size_t i = 0; i < nChars + 2; ++i)
        Serial.print(' ');
    Serial.print("\r> ");
}

static void redrawPrompt(const String& buf)
{
    clearLine(buf.length());
    Serial.print(buf);
}

static inline void printPrompt()
{
    Serial.print("> ");
}

// Utility: set a custom speed profile quickly
// Set a custom speed profile quickly (max speed + acceleration)
static inline void setCustomProfile(PositionController& pc, float vmax, float amax)
{
    pc.setMaxSpeed(vmax);
    pc.setAcceleration(amax);
}

// Schedule the final gentle "Touch" move to the exact target
static void planTouch(uint8_t idx, int32_t finalSteps)
{
    if (!gPC[idx])
        return;

    // Gentle profile for precise approach
    setCustomProfile(*gPC[idx], kTouchMaxSpeed, kTouchAccel);

    // Optional: enable HYBRID just for the final touch window (comment out if not desired)
    // gPC[idx]->setControlMode(ControlMode::HYBRID);

    gPC[idx]->moveToSteps(finalSteps, MovementType::SHORT_RANGE, ControlMode::OPEN_LOOP);

    // After Touch completes, restore your normal completion callback so stable is persisted
    gPC[idx]->attachOnComplete(onMovementComplete);
}

// Callback executed when the Rapid phase reaches the pre-target
static void onRapidArrived()
{
    if (!gPendingTouch)
        return;

    const uint8_t idx = gTouchMotorIdx;
    const int32_t tgt = gTouchFinalSteps;

    gPendingTouch = false;

    // Plan the gentle Touch phase now
    planTouch(idx, tgt);
    Serial.printf("[CHAIN] Rapid arrived -> Touch scheduled (M%d)\r\n", idx + 1);
}

#pragma endregion

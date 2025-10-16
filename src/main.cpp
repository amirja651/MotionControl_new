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

//---------- Used in onHomingComplete ----
// Define globals for chaining
static volatile uint8_t gCurrentMotor   = 0;
static volatile double  gTargetPosition = 0.0f;

// --- Chaining state (Rapid -> Touch) ---
static volatile int32_t gTouchFinalSteps = 0;

// Runtime state
volatile bool gVoltageDrop = false;

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
static constexpr int32_t kTouchWindowStep = 10;     // final window near target in steps
static constexpr double  kTouchWindowDeg  = 1.0;    // final window near target in degrees
static constexpr float   kFastMaxSpeed    = 16000;  // steps/s for Rapid
static constexpr float   kFastAccel       = 24000;  // steps/s^2 for Rapid
static constexpr float   kTouchMaxSpeed   = 600;    // steps/s for Touch
static constexpr float   kTouchAccel      = 1200;   // steps/s^2 for Touch

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

// Lead pitch (µm per revolution) — configurable at runtime
double gLeadPitchUm = kDefaultLeadPitchUm;

static const char* kKeyPitchUm = "sys.pitch_um";

#pragma endregion

#pragma region "Forward declarations"
// ---------- Forward declarations (place before setup()) ----------
static String keyStable(uint8_t i);
static String keyMode(uint8_t i);

static inline bool isLinear(uint8_t idx);
static inline bool isRotary(uint8_t idx);

static void    applyUnitDefaults(uint8_t idx);
static bool    seedCurrentStepWithEncoder(uint8_t idx, int32_t& seedSteps);
static void    saveStableSteps(uint8_t idx, int32_t steps);
static int32_t loadStableSteps(uint8_t idx);

static void onVoltageDropISR();
static void onMovementComplete();
static void configureSpeedByDistanceSteps(PositionController& pc, int32_t current, int32_t target);

// CLI handlers
static void handleMotor(cmd* c);
static void handleMove(cmd* c);
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
static bool readEncoderPulses(uint8_t idx, double& pulses);
static bool readEncoderPulses(uint8_t idx, double& pulses, double& ton_us, double& toff_us);

// Used for handleMove() method
static void onPendingTargetSteps();

// --- Two-phase motion (Rapid -> Touch) declarations ---
static void        onRapidArrived();  // raw function pointer (no lambda)
static inline void setCustomProfileSpeed(PositionController& pc, float vmax, float amax);

// Clear the current line and unhide the prompt
static void        clearLine(size_t nChars);
static void        redrawPrompt(const String& buf);
static inline void printPrompt();

// Enable only the selected encoder to reduce ISR load, then try to read it
void enableSelectedEncoder(uint8_t enc);

static bool validateNParameter(const Argument& aN);
static bool validatePParameter(const Argument& aP);

inline float clampAngle(float angleDeg);
inline float normalizeAndClampAngle(float angleDeg);

#pragma endregion

#pragma region "Setup and Loop"
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
    Serial.printf("Stack overflow checking %s enabled\r\n", txt);

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
    Serial.printf("TMC5160 init:\r\n");
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        // gDriver[i].begin();
        //  Initialize TMC5160 drivers
        if (!gDriver[i].begin())
        {
            Serial.printf("--- DRV%d init ❌ failed\r\n", i + 1);
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
                Serial.printf("--- DRV%d ⚠️ not responding\r\n", i + 1);
            }
            else
            {
                Serial.printf("--- DRV%d is 👍 OK\r\n", i + 1);
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
    Serial.printf("Position Controller init:\r\n");
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        applyUnitDefaults(i);
        // With/without encoder pointer: we pass encoder for all; hybrid optional
        gPC[i] = new PositionController(i, gDriver[i], gDirMux, DriverPins::STEP[i], DriverPins::EN[i], gEnc[i]);
        if (!gPC[i]->begin())
        {
            Serial.printf("--- M%d init ❌ failed\r\n", i + 1);
        }
        else
        {
            Serial.printf("--- M%d initialized\r\n", i + 1);
        }
    }

    // RT task for all controllers
    initializePositionControllers();
    startPositionControlSystem();
    for (uint8_t i = 0; i < kMotorCount; ++i)
        gPC[i]->enable();

    // Movement-complete callback (persist target as stable)
    // for (uint8_t i = 0; i < kMotorCount; ++i)
    //{
    //    gPC[i]->attachOnComplete(onMovementComplete);
    //}

    // Voltage monitor
    // Initialize quick monitor
    if (gVmon.begin())
    {
        gVmon.setHysteresis(50);  // Optional: ±50 window
        gVmon.onDrop(onVoltageDropISR);
        // Serial.printf("Voltage monitor initialized!\r\n");
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
        if (gPC[i]->isEnabled())
        {
            gEnc[i].handleInAbsenceOfInterrupt();
            // gPC[i]->handleMovementComplete();
        }
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

#pragma endregion

#pragma region "Save and Load"
// Keys: "m<idx>.stable_steps", "m<idx>.mode", "sys.pitch_um"
static String keyStable(uint8_t i)
{
    return String("m") + String(i) + ".stable_steps";
}
static String keyMode(uint8_t i)
{
    return String("m") + String(i) + ".mode";
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

#pragma endregion

#pragma region "CLI handlers"
// ---------- CLI handlers ----------
static void handleMotor(cmd* c)
{
    Command  cmd(c);  // Create wrapper object
    Argument aN = cmd.getArg("n");
    (void)validateNParameter(aN);
}

static bool validateNParameter(const Argument& aN)
{
    if (aN.isSet())
    {
        const int nv = aN.getValue().toInt();
        if (nv < 1 || nv > kMotorCount)
        {
            Serial.printf("[ERR] M%d id out of range (1..%d) ❌\r\n", nv, kMotorCount);
            return false;
        }

        gCurrentMotor = static_cast<uint8_t>(nv - 1);

        // ---- Make unit defaults consistent with motor type ----
        applyUnitDefaults(gCurrentMotor);

        Serial.printf("[CLI] Current motor = %d ℹ️\r\n", nv);
        return true;
    }
    else
    {
        Serial.printf("[ERR] missing argument n=<motor index> (valid range: 1-%d) ❌\r\n", kMotorCount);
        return false;
    }
}

static bool validatePParameter(const Argument& aP)
{
    if (aP.isSet())
    {
        gTargetPosition = aP.getValue().toDouble();
        return true;
    }
    else
    {
        Serial.printf("[ERR] missing argument p=<position> (µm for linear, deg for rotary) ❌\r\n");
        return false;
    }
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
    Serial.printf("[STOP] Motor%d\r\n", n + 1);
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
    Serial.printf("[EN] Motor%d enabled\r\n", n + 1);
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
    Serial.printf("[EN] Motor%d disabled\r\n", n + 1);
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
    int32_t seedSteps;
    if (!seedCurrentStepWithEncoder(n, seedSteps))
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
    configureSpeedByDistanceSteps(*gPC[n], cur, stable);
    gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE);
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

static void handleMove(cmd* c)
{
    Command  cmd(c);  // Create wrapper object
    Argument aN = cmd.getArg("n");
    Argument aP = cmd.getArg("p");

    // ---- Parse motor index argument ----
    (void)validateNParameter(aN);

    // ---- Resolve motor index (default: current selection) ----
    uint8_t n = gCurrentMotor;

    // ---- Parse target argument ----
    if (!validatePParameter(aP))
        return;

    if (!gPC[n])
    {
        Serial.printf("[ERR] M%d controller not initialized ❌\r\n", n + 1);
        return;
    }

    if (!gPC[n]->isEnabled())
    {
        Serial.printf("[ERR] M%d is disabled ❌\r\n", n + 1);
        return;
    }

    int32_t stableWithClamp = 0;
    int32_t stable          = loadStableSteps(n);
    double  deg             = UnitConverter::convertFromSteps(stable).TO_DEGREES;

    // ---- Load stable; for rotary clamp to [0.01°, 359.955°] ----
    if (isRotary(n))
    {
        deg = normalizeAndClampAngle(deg);

        stableWithClamp = UnitConverter::convertFromDegrees(deg).TO_STEPS;

        // print
        Serial.printf("[INFO] Stable before=%ld steps, after=%ld steps (%.2f°)\r\n", static_cast<long>(stable), static_cast<long>(stableWithClamp), deg);
        stable = stableWithClamp;
    }
    else
    {
        Serial.printf("[INFO] Stable: %ld steps (%.2f°)\r\n", static_cast<long>(stable), deg);
    }

    int32_t seedSteps;
    if (!seedCurrentStepWithEncoder(n, seedSteps))
        return;

    // ---- If not at stable, go to stable first (single move) ----
    if (seedSteps != stable)
    {
        configureSpeedByDistanceSteps(*gPC[n], seedSteps, stable);

        gPC[n]->attachOnComplete(onPendingTargetSteps);

        if (gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE))
        {
            Serial.printf("[INFO] Homing to stable first: stable (%ld steps), current (%ld steps)\r\n", static_cast<long>(stable), static_cast<long>(seedSteps));

            /*
            [CLI] Current motor = 2 ℹ️
            [INFO] Stable before=3556 steps, after=3556 steps (100.01°)
            [INFO] Seed current position:
                microsteps : 12800 steps
                turns      : 0
                encoder    : 99.67°
                seedSteps  : 3544 steps
                seedAngle  : 99.68°
            [INFO] Homing to stable first: stable (3556 steps), current (3544 steps)
            */
        }
        else
        {
            Serial.println("[INFO] homing-to-stable rejected ⚠️");
            onPendingTargetSteps();
        }

        return;
    }

    onPendingTargetSteps();
}

#pragma endregion

#pragma region "CLI"
// ---------- Setup & Loop ----------
static void attachCli()
{
    initializeCLI();

    cmdMotor.setCallback(handleMotor);
    cmdMove.setCallback(handleMove);
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
                configureSpeedByDistanceSteps(*gPC[n], cur, stable);
                gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE);
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

            double pos_f = 0.0, ton = 0.0, toff = 0.0;
            bool   enabled = readEncoderPulses(n, pos_f, ton, toff);

            if (enabled)
            {
                const float encoderAngle = UnitConverter::convertFromPulses(pos_f).TO_DEGREES;
                Serial.print(F("  Encoder Position: "));
                Serial.print(encoderAngle);
                Serial.print(F("° ("));
                Serial.print(pos_f, 0);
                Serial.println(F(" pulses)"));
            }

            Serial.printf("  dt=%u ms\r\n", dt);
            printPrompt();
            Serial.print(inputBuffer);
            continue;
        }

        // J
        if (upper == kHotkeys[(int)CommandKey::J][0])
        {
            const uint8_t n = gCurrentMotor;
            if ((n + 1) < 1 || (n + 1) > kMotorCount)
            {
                Serial.printf("[ERR] M%d id out of range (1..4) ❌\r\n", n + 1);
                return;
            }
            // ---- Load stable; for rotary clamp to [0.05°, 359.95°] ----
            int32_t stableWithClamp = 0;
            int32_t stable          = loadStableSteps(n);
            double  deg             = UnitConverter::convertFromSteps(stable).TO_DEGREES;
            if (isRotary(n))
            {
                if (deg < 0.01)
                    deg = 0.01;
                if (deg > 359.955)
                    deg = 359.955;
                stableWithClamp = UnitConverter::convertFromDegrees(deg).TO_STEPS;
                // print
                Serial.printf("[INFO] Stable:\r\n");
                Serial.printf("--- before %ld steps\r\n", static_cast<long>(stable));
                Serial.printf("--- after clamp %ld steps or %.2f°\r\n", static_cast<long>(stableWithClamp), deg);
                stable = stableWithClamp;
            }
            else
            {
                Serial.printf("[INFO] Stable: %ld steps (%.2f°)\r\n", static_cast<long>(stable), deg);
            }

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

#pragma region "Read Encoder"
// Read the current position from the encoder (degrees or micrometers)
static bool readEncoderPulses(uint8_t idx, double& pulses)
{
    // Enable only the selected encoder to reduce ISR load, then try to read it
    enableSelectedEncoder(idx);
    double ton_us = 0, toff_us = 0;
    (void)gEnc[idx].tryGetPosition(pulses, ton_us, toff_us);
    delay(300);
    bool response = gEnc[idx].tryGetPosition(pulses, ton_us, toff_us);
    if (!response)
    {
        Serial.printf("[ERR] ENC%d not responding or no data ⚠️\r\n", idx + 1);
    }
    return response;
}
static bool readEncoderPulses(uint8_t idx, double& pulses, double& ton_us, double& toff_us)
{
    // Enable only the selected encoder to reduce ISR load, then try to read it
    enableSelectedEncoder(idx);
    (void)gEnc[idx].tryGetPosition(pulses, ton_us, toff_us);
    delay(300);
    bool response = gEnc[idx].tryGetPosition(pulses, ton_us, toff_us);
    if (!response)
    {
        Serial.printf("[ERR] ENC%d not responding or no data ⚠️\r\n", idx + 1);
    }
    return response;
}

// Enable only the selected encoder to reduce ISR load, then try to read it
void enableSelectedEncoder(uint8_t idx)
{
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        if (i == idx)
            gEnc[i].enable();
        else
            gEnc[i].disable();
    }
}
#pragma endregion

#pragma region "Helpers"
// ---------- Helpers ----------
inline float clampAngle(float angleDeg)
{
    return fminf(fmaxf(angleDeg, 0.01f), 359.955f);
}

inline float normalizeAndClampAngle(float angleDeg)
{
    // Normalize to range 0..360
    while (angleDeg < 0.0f)
        angleDeg += 360.0f;
    while (angleDeg >= 360.0f)
        angleDeg -= 360.0f;

    // Now the final clamping
    return fminf(fmaxf(angleDeg, 0.01f), 359.955f);
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

#pragma endregion

#pragma region "Callback"
static void onVoltageDropISR()
{
    gVoltageDrop = true;
}

// Callback executed when the Rapid phase reaches the pre-target
static void onPendingTargetSteps()
{
    const uint8_t idx = gCurrentMotor;
    double        tgt = gTargetPosition;

    // ---- Compute target steps based on motor type ----
    int32_t pendingTargetSteps = 0;
    if (isLinear(idx))
    {
        // p in micrometers
        pendingTargetSteps = UnitConverter::convertFromMicrometers(tgt).TO_STEPS;
    }
    else
    {
        // p in degrees
        tgt                = normalizeAndClampAngle(tgt);
        pendingTargetSteps = UnitConverter::convertFromDegrees(tgt).TO_STEPS;
    }

    // ---- Motor two-phase motion: Rapid -> Touch (window) ----
    const int32_t currentSteps = gPC[idx]->getCurrentSteps();  // after seeding: equals 'stable'
    const int32_t dStep        = abs(pendingTargetSteps - currentSteps);

    if (dStep > kTouchWindowStep)
    {
        // Phase 1: Rapid to pre-target (target -/+ window)
        const int32_t dir      = (pendingTargetSteps >= currentSteps) ? +kTouchWindowStep : -kTouchWindowStep;
        const int32_t preSteps = pendingTargetSteps - dir * kTouchWindowStep;

        // Persist start in case of power drop
        // saveStableSteps(n, currentSteps);

        // Fast profile for Rapid
        setCustomProfileSpeed(*gPC[idx], kFastMaxSpeed, kFastAccel);

        // IMPORTANT: set Rapid-arrival callback (raw function pointer)
        gPC[idx]->attachOnComplete(onRapidArrived);
        gTouchFinalSteps = pendingTargetSteps;

        // Queue Rapid move and chain Touch via onRapidArrived()
        if (gPC[idx]->moveToSteps(preSteps, MovementType::LONG_RANGE))
        {
            Serial.printf("[MOVE] Rapid to pre-target (deg window=%.2f)\r\n", kTouchWindowDeg);
        }
        else
        {
            Serial.printf("[ERR] Rapid move rejected ❌\r\n");
        }

        return;  // Touch will be scheduled in onRapidArrived()
    }

    // Small distance: single-phase Touch only
    setCustomProfileSpeed(*gPC[idx], kTouchMaxSpeed, kTouchAccel);

    // ---- Linear (or small rotary): single-phase with distance-based profile ----
    configureSpeedByDistanceSteps(*gPC[idx], currentSteps, pendingTargetSteps);

    // Persist start in case of power drop (store the start point)
    // saveStableSteps(n, currentSteps);

    // After Touch completes, restore your normal completion callback so stable is persisted
    gPC[idx]->attachOnComplete(onMovementComplete);

    // Queue final move
    if (gPC[idx]->moveToSteps(pendingTargetSteps, MovementType::MEDIUM_RANGE))
    {
        if (isLinear(idx))
        {
            Serial.printf("[MOVE] linear: target=%.3f µm (steps %ld)\r\n", tgt, static_cast<long>(pendingTargetSteps));
        }
        else
        {
            Serial.printf("[MOVE] rotary: target=%.3f° (steps %ld)\r\n", tgt, static_cast<long>(pendingTargetSteps));
        }
    }
    else
    {
        Serial.println("[ERR] move command rejected ❌");
    }
}

static void onRapidArrived()
{
    const uint8_t idx = gCurrentMotor;
    const int32_t tgt = gTouchFinalSteps;

    if (!gPC[idx])
        return;

    // Gentle profile for precise approach
    setCustomProfileSpeed(*gPC[idx], kTouchMaxSpeed, kTouchAccel);

    // After Touch completes, restore your normal completion callback so stable is persisted
    gPC[idx]->attachOnComplete(onMovementComplete);

    gPC[idx]->moveToSteps(tgt, MovementType::SHORT_RANGE);

    Serial.printf("[CHAIN] Rapid arrived -> Touch scheduled (M%d)\r\n", idx + 1);
}

static void onMovementComplete()
{
    const uint8_t idx = gCurrentMotor;

    // When a motor reaches target, persist the target as stable position
    if (!gPC[idx])
        return;

    const int32_t tgt = gPC[idx]->getTargetSteps();
    saveStableSteps(idx, tgt);

    Serial.printf("[CHAIN] Movement completed 🎯 -> Stable steps saved (M%d)\r\n", idx + 1);
}

// Set a custom speed profile quickly (max speed + acceleration)
static inline void setCustomProfileSpeed(PositionController& pc, float vmax, float amax)
{
    pc.setMaxSpeed(vmax);
    pc.setAcceleration(amax);
}

// Compute distance and configure speed/accel profile accordingly
static void configureSpeedByDistanceSteps(PositionController& pc, int32_t current, int32_t target)
{
    const int32_t d = abs(target - current);
    // Distance-type thresholds and speed tables are embedded in PositionController lean
    pc.configureSpeedByDistanceSteps(d);
}

// Read encoder once and set current step reference for the controller
static bool seedCurrentStepWithEncoder(uint8_t idx, int32_t& seedSteps)
{
    if (!gPC[idx])
        return false;

    const int32_t curSteps = gPC[idx]->getCurrentSteps();
    seedSteps              = curSteps;
    const int32_t ms       = UnitConverter::getDefaultMicrosteps();
    int32_t       turns    = isLinear(idx) ? (curSteps / ms) : 0;  // rotary → force 0 turns

    double pos_f = 0.0;
    if (!readEncoderPulses(idx, pos_f))
    {
        // If curSteps == 0, the encoder could be parked at the zero index,
        // which may cause it to report either zero or an undefined value after startup.
        if (curSteps != 0)
        {
            Serial.printf("[ERR] seed not successful for M%d ❌\r\n", idx + 1);
            return false;
        }
    }

    // Convert encoder pulses (0..4095) to steps within 1 revolution
    const int32_t steps = UnitConverter::convertFromPulses(pos_f).TO_STEPS;
    const float   deg   = UnitConverter::convertFromPulses(pos_f).TO_DEGREES;

    // Rebuild absolute seed:
    //  - linear:  (turns * ms) + steps
    //  - rotary:  (    0 * ms) + steps
    seedSteps               = (turns * ms) + steps;
    const float seedDegrees = UnitConverter::convertFromSteps(seedSteps).TO_DEGREES;

    // Set precise current position before any movement
    gPC[idx]->setCurrentPosition(seedSteps);

    // print
    /*
    Serial.printf("[INFO] Seed current position:\r\n");
    Serial.printf("    microsteps : %d steps\r\n", ms);
    Serial.printf("    turns      : %d\r\n", turns);
    Serial.printf("    encoder    : %.2f°\r\n", deg);
    Serial.printf("    seedSteps  : %d steps\r\n", seedSteps);
    Serial.printf("    seedAngle  : %.2f°\r\n", seedDegrees);
    */
    return true;
}

#pragma endregion

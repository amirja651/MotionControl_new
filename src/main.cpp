#pragma region defin_code
#include <Arduino.h>
#include <CLIManager.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_task_wdt.h>

#include "CommandHistory.h"
#include "DirMultiplexer.h"
#include "MAE3Encoder.h"
#include "PositionController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include "UnitConverter.h"
#include "VoltageMonitor.h"
#include "esp_log.h"
#include "mae3_encoder.hpp"

CommandHistory<16> history;
const char*        commands[] = {"Q", "L", "K", "J"};
enum class CommandKey : uint8_t
{
    Q = 0,
    L = 1,
    K = 2,
    J = 3,
};

// Variables for store in memory ///////////////////////////
enum class VoltageStatus : uint8_t
{
    VOLTAGE_NORMAL,
    VOLTAGE_DROPPED
};

ControlMode control_mode = ControlMode::OPEN_LOOP;
bool        control_save = false;

std::int32_t origin_pulses = 0;
int32_t      origin_turn   = 0;  // For rotary motors always zero
bool         origin_save   = false;

std::int32_t voltageDrop_pulses = 0;
int32_t      voltageDrop_turn   = 0;  // For rotary motors always zero
bool         voltageDrop_save   = false;

VoltageStatus voltage_status = VoltageStatus::VOLTAGE_NORMAL;

std::int32_t reached_pulses = 0;
int32_t      reached_turns  = 0;  // For rotary motors always zero
bool         reached_save   = false;

std::int32_t loaded_pulses = 0;
int32_t      loaded_turns  = 0;  // For rotary motors always zero

///////////////////////////////////////////////////////////

VoltageMonitor voltageMonitor(VoltageMonitorPins::POWER_3_3, VoltageMonitor::MonitorMode::DIGITAL_, 0, 10);

// Driver status tracking
static bool    driverEnabled[4] = {false, false, false, false};
TMC5160Manager driver[4]        = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

// Direction multiplexer for stepper motor direction control
DirMultiplexer dirMultiplexer(MultiplexerPins::S0, MultiplexerPins::S1, MultiplexerPins::DIR);

// MAE3 Encoders for position feedback (read-only, not used for control)
mae3::GpioConfig pins[4] = {
    {36, true, false, false},
    {39, true, false, false},
    {34, true, false, false},
    {35, true, false, false},
};

mae3::Mae3Encoder encoderMae3[4] = {mae3::Mae3Encoder(0, pins[0]), mae3::Mae3Encoder(1, pins[1]), mae3::Mae3Encoder(2, pins[2]), mae3::Mae3Encoder(3, pins[3])};
// Position controllers for precise movement control
PositionController positionController[4] = {PositionController(0, driver[0], dirMultiplexer, DriverPins::STEP[0], DriverPins::EN[0], encoderMae3[0]),
                                            PositionController(1, driver[1], dirMultiplexer, DriverPins::STEP[1], DriverPins::EN[1], encoderMae3[1]),
                                            PositionController(2, driver[2], dirMultiplexer, DriverPins::STEP[2], DriverPins::EN[2], encoderMae3[2]),
                                            PositionController(3, driver[3], dirMultiplexer, DriverPins::STEP[3], DriverPins::EN[3], encoderMae3[3])};

static constexpr std::int32_t SPI_CLOCK            = 1000000;  // 1MHz SPI clock
TaskHandle_t                  serialReadTaskHandle = NULL;

static uint8_t currentIndex = 1;  // Current driver index

// K key press timing
static std::int32_t lastKPressTime = 0;  // Timestamp of last K key press

// Current control parameters
static constexpr std::int32_t MIN_RMS_CURRENT = 100;  // Minimum RMS current in mA
static constexpr std::int32_t MAX_RMS_CURRENT = 500;  // Maximum RMS current in mA

static constexpr uint8_t MIN_IRUN = 1;   // Minimum IRUN value (1/32 of RMS current)
static constexpr uint8_t MAX_IRUN = 31;  // Maximum IRUN value (31/32 of RMS current)

static constexpr uint8_t MIN_IHOLD = 1;   // Minimum IHOLD value (1/32 of RMS current)
static constexpr uint8_t MAX_IHOLD = 31;  // Maximum IHOLD value (31/32 of RMS current)

static constexpr std::int32_t CURRENT_INCREMENT = 10;  // Current change increment
static constexpr uint8_t      IRUN_INCREMENT    = 1;   // IRUN change increment
static constexpr uint8_t      IHOLD_INCREMENT   = 1;   // IHOLD change increment

bool voltageMonitorFirstTime = false;

float global_target = 0.0;

Preferences prefs;
#pragma endregion

#pragma region function_declarations
// Function declarations
void               storeToMemory();
std::int32_t       loadOriginPosition();
void               loadControlMode();
void               loadPosition();
void               printAllOriginPositions();
void               clearLine();
void               setMotorId(String motorId);
void               serialReadTask(void* pvParameters);
void               checkDifferenceCorrection();
void               onVoltageDrop();
void               linearProcess(float targetUMeters);
void               rotationalProcess(float targetAngle);
bool               isLinearMotor(uint8_t index);
bool               isRotationalMotor(uint8_t index);
void               disable();
void               enable();
void               stop();
void               showPositionStatus();
const char*        toString(VoltageStatus status);
const char*        toString(ControlMode m);
const char*        toString(Direction m);
String             makeKey(int motorIndex, const char* suffix);
String             isSave(size_t result);
inline ControlMode safeModeFromInt(int v, ControlMode fallback = ControlMode::HYBRID);
bool               isMoving();
static void        printConversionTable(float inputValue, const ConvertValues& cvfd, const ConvertValues& cvfp, const ConvertValues& cvfs, const ConvertValues& cvfm);

#pragma endregion

#pragma region setup_and_loop
void setup()
{
    // Initialize SPI
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(115200);

    esp_log_level_set("*", ESP_LOG_INFO);

    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

#ifdef CONFIG_FREERTOS_CHECK_STACKOVERFLOW
    log_d("Stack overflow checking enabled");
#else
    log_d("Stack overflow checking **NOT** enabled");
#endif

    // Initialize Preferences for storing motor origin positions
    if (!prefs.begin("motion_control", false))  // false = read/write mode
    {
        log_e("Failed to initialize Preferences");
    }
    else
    {
        log_d("Preferences successfully initialized");
    }

    // Initialize CLI
    initializeCLI();

    // Initialize and print system diagnostics
    SystemDiagnostics::printSystemInfo();
    SystemDiagnostics::printSystemStatus();

    // for disable all drivers pins - for avoid conflict in SPI bus
    // Initialize CS pins and turn them off with safety checks
    for (uint8_t index = 0; index < 4; index++)
    {
        pinMode(DriverPins::CS[index], OUTPUT);
        digitalWrite(DriverPins::CS[index], HIGH);
    }

    // Initialize TMC5160 drivers
    for (uint8_t index = 0; index < 4; index++)
    {
        // Create driver
        driver[index].begin();

        // Test connection for each driver
        if (driver[index].testConnection(true))
        {
            driverEnabled[index] = true;
            driver[index].configureDriver_All_Motors(true);
        }

        // Initialize position controllers and encoders
        if (driverEnabled[index])
        {
            positionController[index].begin();
            encoderMae3[index].init();
            encoderMae3[index].disable();  // Enable encoder for reading
        }
    }
    encoderMae3[0].init();
    encoderMae3[0].enable();
    // Initialize position control system
    initializePositionControllers();
    startPositionControlSystem();
    setMotorId("2");

    // Add longer delay before accessing preferences to ensure stability
    delay(100);
    printAllOriginPositions();
    loadPosition();
    delay(100);

    esp_task_wdt_init(15, true);  // Increased timeout to 15 seconds
    esp_task_wdt_add(NULL);       // Add the current task (setup)

    // Initialize quick monitor
    if (voltageMonitor.begin())
    {
        voltageMonitorFirstTime = true;
        voltageMonitor.onDrop(onVoltageDrop);
        log_d("Voltage monitor initialized!");
    }

    // Create serial read task with larger stack
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 8192, NULL, 2, &serialReadTaskHandle, 0);
    esp_task_wdt_add(serialReadTaskHandle);  // Register with WDT

    log_d("Position control system initialized");
    log_i("Use 'L' to show position status");
}

void loop()
{
    // Handle movement complete outside ISR
    encoderMae3[currentIndex].handleInAbsenceOfInterrupt();
    positionController[currentIndex].handleMovementComplete();
    voltageMonitor.update();

    // Example: Check for drop detection manually
    if (voltageMonitor.wasDropDetected())
    {
        log_d("Power drop was detected!");
        voltageMonitor.resetDropDetection();
    }

    voltageMonitorFirstTime = !voltageMonitor.isVoltageOK();

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
}
#pragma endregion

#pragma region memory

void storeToMemory()
{
    noInterrupts();

    if (control_save)
    {
        control_save = false;

        String key_cm = makeKey(currentIndex, "cm");  // control mode
        size_t w      = prefs.putInt(key_cm.c_str(), static_cast<int>(control_mode));
        vTaskDelay(pdMS_TO_TICKS(20));

        log_d("\n⚙️ Motor [%d]\n"
              "   • Key     : %d\n"
              "   • Value   : %d\n"
              "   • Success : %s\n",
              currentIndex + 1,
              key_cm.c_str(),
              static_cast<int>(control_mode),
              isSave(w));
    }

    if (origin_save)
    {
        origin_save = false;

        String key_op = makeKey(currentIndex, "op");  // origin pulses
        size_t w1     = prefs.putUInt(key_op.c_str(), origin_pulses);
        vTaskDelay(pdMS_TO_TICKS(20));

        String key_ot = makeKey(currentIndex, "ot");  // origin turns
        size_t w2     = prefs.putInt(key_ot.c_str(), origin_turn);
        vTaskDelay(pdMS_TO_TICKS(20));

        log_d("\n⚙️ Motor [%d]\n"
              "   • Key     : %d\n"
              "   • Value   : %u\n"
              "   • Success : %s\n"
              "    -------------\n"
              "   • Key     : %d\n"
              "   • Value   : %u\n"
              "   • Success : %s\n",
              currentIndex + 1,
              key_op.c_str(),
              origin_pulses,
              isSave(w1),
              key_ot.c_str(),
              origin_turn,
              isSave(w2));
    }

    if (voltageDrop_save)
    {
        voltageDrop_save = false;

        // pulses (UINT)
        String key_vdp = makeKey(currentIndex, "vdp");
        size_t w1      = prefs.putUInt(key_vdp.c_str(), voltageDrop_pulses);
        vTaskDelay(pdMS_TO_TICKS(20));

        // turns (INT)
        String key_vdt = makeKey(currentIndex, "vdt");
        size_t w2      = prefs.putInt(key_vdt.c_str(), voltageDrop_turn);
        vTaskDelay(pdMS_TO_TICKS(20));

        // voltage status (INT)
        String key_vs = makeKey(currentIndex, "vs");
        size_t w3     = prefs.putInt(key_vs.c_str(), static_cast<int>(voltage_status));
        vTaskDelay(pdMS_TO_TICKS(20));

        log_d("\n⚙️ Motor [%d]\n"
              "   • Key     : %d\n"
              "   • Value   : %u\n"
              "   • Success : %s\n"
              "    -------------\n"
              "   • Key     : %d\n"
              "   • Value   : %d\n"
              "   • Success : %s\n"
              "    -------------\n"
              "   • Key     : %d\n"
              "   • Value   : %d\n"
              "   • Success : %s\n",
              currentIndex + 1,
              key_vdp.c_str(),
              voltageDrop_pulses,
              isSave(w1),
              key_vdt.c_str(),
              voltageDrop_turn,
              isSave(w2),
              key_vs.c_str(),
              static_cast<int>(voltage_status),
              isSave(w3));
    }

    if (reached_save)
    {
        reached_save = false;

        // pulses (UINT)
        String key_rp = makeKey(currentIndex, "rp");
        size_t w1     = prefs.putUInt(key_rp.c_str(), reached_pulses);
        vTaskDelay(pdMS_TO_TICKS(20));

        // turns (INT)
        String key_rt = makeKey(currentIndex, "rt");
        size_t w2     = prefs.putInt(key_rt.c_str(), reached_turns);
        vTaskDelay(pdMS_TO_TICKS(20));

        // voltage status (INT)
        String key_vs = makeKey(currentIndex, "vs");
        size_t w3     = prefs.putInt(key_vs.c_str(), static_cast<int>(voltage_status));
        vTaskDelay(pdMS_TO_TICKS(20));

        log_d("\n⚙️ Motor [%d]\n"
              "   • Key     : %d\n"
              "   • Value   : %u\n"
              "   • Success : %s\n"
              "    -------------\n"
              "   • Key     : %d\n"
              "   • Value   : %d\n"
              "   • Success : %s\n"
              "    -------------\n"
              "   • Key     : %d\n"
              "   • Value   : %d\n"
              "   • Success : %s\n",
              currentIndex + 1,
              key_rp.c_str(),
              reached_pulses,
              isSave(w1),
              key_rt.c_str(),
              reached_turns,
              isSave(w2),
              key_vs.c_str(),
              static_cast<int>(voltage_status),
              isSave(w3));
    }

    interrupts();

    // Clear the "absence of interrupt" flag (post-write)
    encoderMae3[currentIndex].setInAbsenceOfInterruptFlag(false);
}

std::int32_t loadOriginPosition()
{
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return 0;  // fixed: correct return type
    }

    String       key_op        = makeKey(currentIndex, "op");
    std::int32_t origin_pulses = prefs.getUInt(key_op.c_str(), 0);  // Default to 0 if not found

    log_d("\n⚙️ Motor [%d]\n"
          "   • Origin : %u pulses\n",
          currentIndex + 1,
          origin_pulses);

    return origin_pulses;
}

void loadControlMode()
{
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return;
    }

    String key_cm = makeKey(currentIndex, "cm");  // "m<idx>_cm"
    int    raw    = prefs.getInt(key_cm.c_str(), static_cast<int>(ControlMode::HYBRID));
    control_mode  = safeModeFromInt(raw, ControlMode::HYBRID);

    log_d("\n⚙️ Motor [%d]\n"
          "   • Mode  : %s\n",
          currentIndex + 1,
          toString(control_mode));
}

void loadPosition()
{
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return;
    }

    // Load voltage status
    String key_voltage_status = makeKey(currentIndex, "vs");
    voltage_status            = static_cast<VoltageStatus>(prefs.getInt(key_voltage_status.c_str(), static_cast<int>(VoltageStatus::VOLTAGE_NORMAL)));

    if (voltage_status == VoltageStatus::VOLTAGE_NORMAL)
    {
        String key_reached_pulses = makeKey(currentIndex, "rp");
        loaded_pulses             = prefs.getUInt(key_reached_pulses.c_str(), 0);

        String key_reached_turns = makeKey(currentIndex, "rt");
        loaded_turns             = prefs.getInt(key_reached_turns.c_str(), 0);
    }
    else
    {
        String key_voltageDrop_pulses = makeKey(currentIndex, "vdp");
        loaded_pulses                 = prefs.getUInt(key_voltageDrop_pulses.c_str(), 0);

        String key_voltageDrop_turn = makeKey(currentIndex, "vdt");
        loaded_turns                = prefs.getInt(key_voltageDrop_turn.c_str(), 0);
    }

    log_d("\n⚙️ Motor [%d]\n"
          "   • Status : %s\n"
          "   • Turn   : %d\n"
          "   • Pulses : %u\n",
          currentIndex + 1,
          toString(voltage_status),
          loaded_turns,
          loaded_pulses);
}

void printAllOriginPositions()
{
    log_d("Current origin positions:");

    // Add delay to ensure system stability
    delay(10);

    for (uint8_t i = 0; i < 4; i++)
    {
        String key            = "m" + String(i) + "_or";
        float  originPosition = prefs.getFloat(key.c_str(), 0.0f);
        log_d("Motor %d: %f°", i + 1, originPosition);

        // Small delay between each motor to prevent overload
        delay(5);

        if (isLinearMotor(i))
        {
            String  key2  = "m" + String(i) + "_tu";
            int32_t turns = prefs.getInt(key2.c_str(), 0);
            log_d("Motor %d: %d turns", i + 1, turns);

            if (turns != 0)
            {
                originPosition = turns * PIXEL_SIZE_MM;
                log_d("Motor %d: Origin Position %f°", i + 1, originPosition);
            }
        }
    }

    // Print total number of keys for debugging
    size_t totalKeys = prefs.freeEntries();
    log_d("Total free entries: %d", totalKeys);
}

#pragma endregion

void clearLine()
{
    Serial.print(F("\r"));  // Go to the beginning of the line
    Serial.print(F("                                                                      "
                   "          "));  // About 80 characters
    Serial.print(F("\r"));
}

void setMotorId(String motorId)
{
    // 1. Check that it contains only numbers
    for (size_t i = 0; i < motorId.length(); i++)
    {
        if (!isDigit(motorId[i]))
        {
            log_e("Invalid Motor Id. limit: 1-4");
            return;
        }
    }

    // 2. Convert to integer
    int index = motorId.toInt();
    if (index < 1 || index > 4)
    {
        log_e("Invalid Motor Id. limit: 1-4");
        return;
    }

    currentIndex = index - 1;

    // Initialize position controllers and encoders
    for (uint8_t index = 0; index < 4; index++)
    {
        if (driverEnabled[index])
        {
            encoderMae3[index].disable();  // Enable encoder for reading
        }
    }

    UnitConverter::setDefaultResolution(ENCODER_RESOLUTION);
    UnitConverter::setDefaultMicrometers(LEAD_SCREW_PITCH_UM);

    if (currentIndex > 0)  // Rotary motors
    {
        UnitConverter::setDefaultMotorType(MotorType::ROTATIONAL);
        UnitConverter::setDefaultMicrosteps((MICROSTEPS_64 - 1) * 200);
    }
    else  // Linear motor
    {
        UnitConverter::setDefaultMotorType(MotorType::LINEAR);
        UnitConverter::setDefaultMicrosteps((MICROSTEPS_32 - 1) * 200);
    }

    encoderMae3[currentIndex].enable();
    log_i("Motor %d selected and enabled", currentIndex + 1);
}

void serialReadTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(100);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    String           inputBuffer   = "";
    String           lastInput     = "";

    while (1)
    {
        while (Serial.available())
        {
            char c = Serial.read();

            // Handle escape sequences for arrow keys
            static int escState = 0;  // 0: normal, 1: got '\x1b', 2: got '['
            if (escState == 0 && c == '\x1b')
            {
                escState = 1;
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            if (escState == 1 && c == '[')
            {
                escState = 2;
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            if (escState == 2)
            {
                if (c == 'A')
                {  // Up arrow

                    inputBuffer = history.up();
                    clearLine();
                    Serial.print("> ");
                    Serial.print(inputBuffer);
                    escState = 0;
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
                if (c == 'B')
                {  // Down arrow
                    inputBuffer = history.down();
                    clearLine();
                    Serial.print("> ");
                    Serial.print(inputBuffer);
                    escState = 0;
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
                escState = 0;
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // Handle Enter
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.println();
                    Serial.print(F("# "));
                    Serial.println(inputBuffer.c_str());
                    cli.parse(inputBuffer);
                    history.push(inputBuffer);
                    history.resetCursor();  // go back to the end
                    lastInput   = "";
                    inputBuffer = "";  // Clear the buffer
                }
            }
            else if (c == '\b' || c == 127)  // Handle backspace
            {
                if (inputBuffer.length() > 0)
                {
                    inputBuffer.remove(inputBuffer.length() - 1);
                    Serial.print(F("\b \b"));
                }
            }
            else if (c == commands[static_cast<int>(CommandKey::Q)][0])  // 'Q' go to position from memory
            {
                if (!isMoving())
                {
                    ConvertValues encoderPulses = UnitConverter::convertFromPulses(loaded_pulses);
                    float         target;
                    if (isLinearMotor(currentIndex))
                    {
                        target = encoderPulses.TO_MICROMETERS + (loaded_turns * LEAD_SCREW_PITCH_UM);
                        linearProcess(target);
                    }
                    else
                    {
                        target = encoderPulses.TO_DEGREES;
                        rotationalProcess(target);
                    }
                }
            }
            else if (c == commands[static_cast<int>(CommandKey::L)][0])  // 'L' Show position status
            {
                if (!isMoving())
                    showPositionStatus();
            }
            else if (c == commands[static_cast<int>(CommandKey::K)][0])  // 'K' Show encoder interrupt counters
            {
                if (!isMoving())
                {
                    std::int32_t currentTime  = millis();
                    std::int32_t timeInterval = 0;

                    // Calculate time interval since last K press
                    if (lastKPressTime > 0)
                    {
                        timeInterval = currentTime - lastKPressTime;
                    }

                    Serial.println();
                    Serial.print(F("[Encoder Interrupt Counters] Motor "));
                    Serial.print(currentIndex + 1);
                    Serial.println(F(":"));

                    if (currentIndex >= 4 || !driverEnabled[currentIndex])
                    {
                        log_e("Invalid motor index or driver not enabled");
                    }
                    else
                    {
                        Serial.println(encoderMae3[currentIndex].enable() == mae3::Status::Ok ? F("YES") : F("NO"));
                        if (encoderMae3[currentIndex].enable() == mae3::Status::Ok)
                        {
                            auto  encoderState = positionController[currentIndex].getEncoderState();
                            float encoderAngle = UnitConverter::convertFromPulses(encoderState.position_pulse).TO_DEGREES;
                            Serial.print(F("  Encoder Position: "));
                            Serial.print(encoderAngle);
                            Serial.print(F("° ("));
                            Serial.print(encoderState.position_pulse);
                            Serial.println(F(" pulses)"));
                        }
                        else
                        {
                            log_w("Encoder not enabled");
                        }

                        // Display time interval
                        if (lastKPressTime > 0)
                        {
                            Serial.print(F("  Time since last K press: "));
                            Serial.print(timeInterval);
                            Serial.println(F(" ms"));
                        }
                        else
                        {
                            Serial.println(F("  Time since last K press: First press"));
                        }

                        // Update last K press time
                        lastKPressTime = currentTime;
                    }
                }
            }
            else if (c == commands[static_cast<int>(CommandKey::J)][0])  // 'J' Show encoder interrupt counters
            {
                if (!isMoving())
                {
                    const std::int32_t currentTime  = millis();
                    std::int32_t       timeInterval = 0;

                    if (lastKPressTime != 0)
                    {
                        timeInterval = currentTime - lastKPressTime;  // std::int32_t wrap-safe
                    }

                    Serial.println();
                    Serial.printf("[Encoder Interrupt Counters] Motor %d:\n", currentIndex + 1);

                    if (currentIndex >= 4 || !driverEnabled[currentIndex])
                    {
                        log_e("Invalid motor index or driver not enabled");
                        Serial.println(F("  [Aborted]"));
                        return;
                    }

                    // Snapshot / reads
                    const bool enabled = encoderMae3[currentIndex].enable() == mae3::Status::Ok;

                    Serial.printf("  Encoder Enabled       : %s\n", enabled ? "YES" : "NO");

                    if (enabled)
                    {
                        auto        encoderState = positionController[currentIndex].getEncoderState();
                        const float angle        = UnitConverter::convertFromPulses(encoderState.position_pulse).TO_DEGREES;
                        Serial.printf("  Encoder Position      : %.2f° (%u pulses)\n", angle, encoderState.position_pulse);
                    }
                    else
                    {
                        log_w("Encoder not enabled");
                    }

                    if (lastKPressTime != 0)
                    {
                        Serial.printf("  Time since last J     : %u ms\n", timeInterval);
                    }
                    else
                    {
                        Serial.println(F("  Time since last J     : First press"));
                    }

                    // Update last press time
                    lastKPressTime = currentTime;
                }
            }
            else  // Only add to buffer if not a direct command
            {
                bool isCommand = false;
                for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++)
                {
                    if (c == commands[i][0])
                    {
                        isCommand = true;
                        break;
                    }
                }
                if (!isCommand)
                {
                    inputBuffer += c;  // Add character to buffer
                    Serial.print(c);
                }
            }

            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }

        if (cli.available())
        {
            Command c = cli.getCmd();
            if (c == cmdMotor)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
            }
            else if (c == cmdMove)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                if (c.getArgument("p").isSet())
                {
                    String targetStr = c.getArgument("p").getValue();
                    float  target    = targetStr.toFloat();
                    global_target    = target;

                    if (isLinearMotor(currentIndex))
                    {
                        linearProcess(target);
                    }
                    else
                    {
                        rotationalProcess(target);
                    }
                }
            }
            else if (c == cmdControlMode)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                if (c.getArgument("o").isSet())
                {
                    control_mode = ControlMode::OPEN_LOOP;
                }
                else if (c.getArgument("h").isSet())
                {
                    control_mode = ControlMode::HYBRID;
                }

                control_save = true;
                storeToMemory();
            }
            else if (c == cmdStop)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                stop();
            }
            else if (c == cmdEnable)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                enable();
            }
            else if (c == cmdDisable)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                stop();
                vTaskDelay(pdMS_TO_TICKS(25));
                disable();
            }
            else if (c == cmdCurrentPosition)
            {
                if (!isMoving())
                {
                    if (c.getArgument("n").isSet())
                    {
                        String motorIdStr = c.getArgument("n").getValue();
                        setMotorId(motorIdStr);
                    }
                    int32_t currentSteps = positionController[currentIndex].getCurrentSteps();
                    float   position     = UnitConverter::convertFromSteps(currentSteps).TO_DEGREES;
                    Serial.print(F("CURRENT"));
                    Serial.print(position);
                }
            }
            else if (c == cmdLastPosition)
            {
                if (!isMoving())
                {
                    if (c.getArgument("n").isSet())
                    {
                        String motorIdStr = c.getArgument("n").getValue();
                        setMotorId(motorIdStr);
                    }
                    float target;
                    if (isLinearMotor(currentIndex))
                    {
                        target = UnitConverter::convertFromPulses(loaded_pulses).TO_MICROMETERS + (loaded_turns * ENCODER_RESOLUTION);
                    }
                    else
                    {
                        target = UnitConverter::convertFromPulses(loaded_pulses).TO_DEGREES;
                    }
                    Serial.print(F("LAST"));
                    Serial.print(target);
                }
            }
            else if (c == cmdSave)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);

                    if (encoderMae3[currentIndex].enable() == mae3::Status::Ok)
                    {
                        auto          encoderState  = positionController[currentIndex].getEncoderState();
                        ConvertValues encoderPulses = UnitConverter::convertFromPulses(encoderState.position_pulse);
                        Direction     direction     = encoderState.direction;
                        MotorStatus   motorStatus   = positionController[currentIndex].getMotorStatus();

                        if (motorStatus.currentSteps == 0)
                        {
                            positionController[currentIndex].setCurrentPosition(encoderPulses.TO_STEPS);
                            motorStatus.currentSteps = positionController[currentIndex].getCurrentSteps();
                        }

                        ConvertValues currentMotorSteps = UnitConverter::convertFromSteps(motorStatus.currentSteps);
                        ConvertValues targetMotorSteps  = UnitConverter::convertFromSteps(motorStatus.targetSteps);

                        // Pick readable strings up-front
                        const char* movingStr  = motorStatus.isMoving ? "YES" : "NO";
                        const char* enabledStr = motorStatus.isEnabled ? "YES" : "NO";

                        // Check if this is a linear motor (motor 1) or rotational motor
                        if (isLinearMotor(currentIndex))
                        {
                            // For linear motor, display in micrometers
                            float   diff = currentMotorSteps.TO_MICROMETERS - encoderPulses.TO_MICROMETERS;
                            int32_t turn = currentMotorSteps.TO_TURNS;
                            origin_turn  = turn;

                            log_d("\n⚙️ Motor [%d] (Linear)\n"
                                  "   • Current  : %f µm (%d turns) 💡\n"
                                  "   • Target   : %f µm (%d turns)\n"
                                  "   • Diff     : %f µm\n"
                                  "   • Encoder  : %f µm (%s, %u pulses)\n"
                                  "   • Moving   : %s\n"
                                  "   • Enabled  : %s\n"
                                  "   • Mode     : %s\n",
                                  currentIndex + 1,
                                  currentMotorSteps.TO_MICROMETERS,
                                  currentMotorSteps.TO_TURNS,
                                  targetMotorSteps.TO_MICROMETERS,
                                  targetMotorSteps.TO_TURNS,
                                  diff,
                                  encoderPulses.TO_MICROMETERS,
                                  toString(direction),
                                  encoderState.position_pulse,
                                  movingStr,
                                  enabledStr,
                                  toString(motorStatus.controlMode));
                        }
                        else
                        {
                            // For rotational motors, display in degrees
                            float diff  = currentMotorSteps.TO_DEGREES - encoderPulses.TO_DEGREES;
                            origin_turn = 0;

                            log_d("\n⚙️ Motor [%d] (Rotational)\n"
                                  "   • Current  : %f° (%d turns) 💡\n"
                                  "   • Target   : %f° (%d turns)\n"
                                  "   • Diff     : %f°\n"
                                  "   • Encoder  : %f° (%s, %u pulses)\n"
                                  "   • Moving   : %s\n"
                                  "   • Enabled  : %s\n"
                                  "   • Mode     : %s\n",
                                  currentIndex + 1,
                                  currentMotorSteps.TO_DEGREES,
                                  0,
                                  targetMotorSteps.TO_DEGREES,
                                  0,
                                  diff,
                                  encoderPulses.TO_DEGREES,
                                  toString(direction),
                                  encoderState.position_pulse,
                                  movingStr,
                                  enabledStr,
                                  toString(motorStatus.controlMode));
                        }
                        origin_pulses = encoderState.position_pulse;
                        origin_save   = true;
                    }
                    else
                    {
                        log_w("Encoder not enabled");
                    }
                }
            }
            else if (c == cmdRestart)
            {
                ESP.restart();
            }
            else if (c == cmdShow)
            {
                if (!isMoving())
                {
                    if (c.getArgument("n").isSet())
                    {
                        String motorIdStr = c.getArgument("n").getValue();
                        setMotorId(motorIdStr);
                        showPositionStatus();
                    }
                }
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.print(cli.toString());
                Serial.println();
            }
            else if (c == cmdTest)
            {
                if (c.getArgument("p").isSet())
                {
                    String valueStr = c.getArgument("p").getValue();
                    float  value    = valueStr.toFloat();

                    ConvertValues cvfd = UnitConverter::convertFromDegrees(value);
                    ConvertValues cvfp = UnitConverter::convertFromPulses(value);
                    ConvertValues cvfs = UnitConverter::convertFromSteps(value);
                    ConvertValues cvfm = UnitConverter::convertFromMicrometers(value);

                    Serial.println();
                    printConversionTable(value, cvfd, cvfp, cvfs, cvfm);
                    Serial.println();
                }
            }
        }

        if (cli.errored())
        {
            String cmdError = cli.getError().toString();
            log_e("ERROR: %s", cmdError.c_str());
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void checkDifferenceCorrection()
{
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_task_wdt_reset();

    auto          encoderState      = positionController[currentIndex].getEncoderState();
    ConvertValues encoderPulses     = UnitConverter::convertFromPulses(encoderState.position_pulse);
    MotorStatus   motorStatus       = positionController[currentIndex].getMotorStatus();
    ConvertValues currentMotorSteps = UnitConverter::convertFromSteps(motorStatus.currentSteps);
    float         difference        = encoderPulses.TO_DEGREES - currentMotorSteps.TO_DEGREES;

    if (fabs(difference) > 0.05)
    {
        if (isLinearMotor(currentIndex))
        {
            linearProcess(global_target);
        }
        else
        {
            rotationalProcess(global_target);
        }
    }
    else
    {
        log_i("No movement needed!");
        log_i("anjam shod pooya.");
    }
}

void onVoltageDrop()
{
    log_w("Voltage drop detected - possible glitch");
    if (voltageMonitorFirstTime)
    {
        log_d("Voltage monitor first time, skipping voltage drop");
        return;
    }

    if (isMoving())
    {
        stop();
        voltageDrop_save = true;
        storeToMemory();

        log_w("Motor %d stopped due to voltage drop, position saved", currentIndex + 1);
    }
}

void linearProcess(float targetMicrometers)
{
    MotorStatus   motorStatus = positionController[currentIndex].getMotorStatus();
    ConvertValues motor       = UnitConverter::convertFromSteps(motorStatus.currentSteps);
    voltageDrop_turn          = motor.TO_TURNS;

    // set current position from encoders //////////
    auto          encoderState  = positionController[currentIndex].getEncoderState();
    ConvertValues encoderPulses = UnitConverter::convertFromPulses(encoderState.position_pulse);
    int32_t       microsteps    = UnitConverter::getDefaultMicrosteps();
    int32_t       currentSteps  = encoderPulses.TO_STEPS + (motor.TO_TURNS * microsteps);
    positionController[currentIndex].setCurrentPosition(currentSteps);
    ///////////////////////////////////////////////

    voltageDrop_pulses = encoderState.position_pulse;
    loadControlMode();
    encoderMae3[currentIndex].attachInAbsenceOfInterrupt(storeToMemory);
    positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);
    ConvertValues target      = UnitConverter::convertFromMicrometers(targetMicrometers);
    int32_t       targetSteps = target.TO_STEPS + (target.TO_TURNS * microsteps);

    log_d("\n⚙️ Motor [%d] (Linear)\n"

          "   • Motor        : %d total steps (%d steps, %d turns, %f µm, new: "
          "%d)\n"
          "   • Target       : %d total steps (%d steps, %d turns, %f µm)\n"
          "   • Encoder      : %f° (%f µm)\n"
          "   • Mode         : %s\n"
          "   • Voltage drop : %u pulses (%d turns)\n",

          currentIndex + 1,

          motorStatus.currentSteps,
          motor.TO_STEPS,
          motor.TO_TURNS,
          motor.TO_MICROMETERS,
          currentSteps,

          targetSteps,
          target.TO_STEPS,
          target.TO_TURNS,
          target.TO_MICROMETERS,

          encoderPulses.TO_DEGREES,
          encoderPulses.TO_MICROMETERS,

          toString(control_mode),

          voltageDrop_pulses,
          voltageDrop_turn);

    bool success = positionController[currentIndex].moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, control_mode);

    if (success)
    {
        log_i("🚀 Move queued");
    }
    else
    {
        log_e("❌ Failed to queue movement");
        log_i("anjam shod pooya.");
    }
}

void rotationalProcess(float targetAngle)
{
    if (targetAngle == 0)
        targetAngle = 0.09f;
    else if (targetAngle == 360)
        targetAngle = 359.9955f;

    voltageDrop_turn = 0;

    // set current position from encoders //////////
    auto          encoderState  = positionController[currentIndex].getEncoderState();
    ConvertValues encoderPulses = UnitConverter::convertFromPulses(encoderState.position_pulse);
    std::int32_t  currentSteps  = encoderPulses.TO_STEPS;
    positionController[currentIndex].setCurrentPosition(currentSteps);
    ///////////////////////////////////////////////

    voltageDrop_pulses = encoderState.position_pulse;
    loadControlMode();
    encoderMae3[currentIndex].attachInAbsenceOfInterrupt(storeToMemory);
    positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);
    int32_t targetSteps = UnitConverter::convertFromDegrees(targetAngle).TO_STEPS;

    log_d("\n⚙️ Motor [%d] (Rotational)\n"
          "   • Steps        : %d\n"
          "   • Degrees      : %f°\n"
          "   • Voltage drop : %u pulses\n"
          "   • Turns        : %d\n"
          "   • Mode         : %s\n"
          "   • Target       : %f steps\n",
          currentIndex + 1,
          encoderPulses.TO_STEPS,
          encoderPulses.TO_DEGREES,
          voltageDrop_pulses,
          voltageDrop_turn,
          toString(control_mode),
          targetSteps);

    bool success = positionController[currentIndex].moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, control_mode);

    if (success)
    {
        log_i("🚀 Move queued");
    }
    else
    {
        log_e("❌ Failed to queue movement");
        log_i("anjam shod pooya.");
    }
}

#pragma region helper_functions

bool isLinearMotor(uint8_t index)
{
    return (positionController[index].getMotorType() == MotorType::LINEAR);
}
bool isRotationalMotor(uint8_t index)
{
    return (positionController[index].getMotorType() == MotorType::ROTATIONAL);
}
void disable()
{
    positionController[currentIndex].disable();
}
void enable()
{
    positionController[currentIndex].enable();
}
void stop()
{
    positionController[currentIndex].stop();
}
void showPositionStatus()
{
    auto encoderState = positionController[currentIndex].getEncoderState();

    ConvertValues encoderPulses = UnitConverter::convertFromPulses(encoderState.position_pulse);
    Direction     direction     = encoderState.direction;
    MotorStatus   motorStatus   = positionController[currentIndex].getMotorStatus();

    if (motorStatus.currentSteps == 0)
    {
        positionController[currentIndex].setCurrentPosition(encoderPulses.TO_STEPS);
        motorStatus.currentSteps = positionController[currentIndex].getCurrentSteps();
    }

    ConvertValues currentMotorSteps = UnitConverter::convertFromSteps(motorStatus.currentSteps);
    ConvertValues targetMotorSteps  = UnitConverter::convertFromSteps(motorStatus.targetSteps);

    // Pick readable strings up-front
    const char* movingStr  = motorStatus.isMoving ? "YES" : "NO";
    const char* enabledStr = motorStatus.isEnabled ? "YES" : "NO";

    // Check if this is a linear motor (motor 1) or rotational motor
    if (isLinearMotor(currentIndex))
    {
        // For linear motor, display in micrometers
        int32_t diff_steps   = motorStatus.targetSteps - motorStatus.currentSteps;
        float   diff_encoder = currentMotorSteps.TO_MICROMETERS - encoderPulses.TO_MICROMETERS;
        int32_t turn         = currentMotorSteps.TO_TURNS;
        voltageDrop_turn     = turn;

        log_d("\n⚙️ Motor [%d] (Linear)\n"

              "   • Current      : %d total steps (%d steps, %d turns, %f µm)\n"
              "   • Target       : %d total steps (%d steps, %d turns, %f µm)\n"
              "   • Diff         : %f steps\n"
              "   • Encoder      : %f° (%f µm, diff: %f µm, dir: %s)\n"
              "   • Moving       : %s\n"
              "   • Enabled      : %s\n"
              "   • Mode         : %s\n"
              "   • Voltage drop : %u pulses (%d turns)\n",

              currentIndex + 1,

              motorStatus.currentSteps,
              currentMotorSteps.TO_STEPS,
              currentMotorSteps.TO_TURNS,
              currentMotorSteps.TO_MICROMETERS,

              motorStatus.targetSteps,
              targetMotorSteps.TO_STEPS,
              targetMotorSteps.TO_TURNS,
              targetMotorSteps.TO_MICROMETERS,

              diff_steps,

              encoderPulses.TO_DEGREES,
              encoderPulses.TO_MICROMETERS,
              diff_encoder,
              toString(direction),

              movingStr,
              enabledStr,

              toString(motorStatus.controlMode),

              voltageDrop_pulses,
              voltageDrop_turn);
    }
    else
    {
        // For rotational motors, display in degrees
        float diff       = currentMotorSteps.TO_DEGREES - encoderPulses.TO_DEGREES;
        voltageDrop_turn = 0;

        log_d("\n⚙️ Motor [%d] (Rotational)\n"
              "   • Current  : %f° (%d turns) 💡\n"
              "   • Target   : %f° (%d turns)\n"
              "   • Diff     : %f°\n"
              "   • Encoder  : %f° (%s, %u pulses)\n"
              "   • Moving   : %s\n"
              "   • Enabled  : %s\n"
              "   • Mode     : %s\n",
              currentIndex + 1,
              currentMotorSteps.TO_DEGREES,
              0,
              targetMotorSteps.TO_DEGREES,
              0,
              diff,
              encoderPulses.TO_DEGREES,
              toString(direction),
              encoderState.position_pulse,
              movingStr,
              enabledStr,
              toString(motorStatus.controlMode));
    }
}

const char* toString(VoltageStatus status)
{
    switch (status)
    {
        case VoltageStatus::VOLTAGE_NORMAL:
            return "✅ NORMAL";
        case VoltageStatus::VOLTAGE_DROPPED:
            return "⚡ DROPPED";
        default:
            return "❓ UNKNOWN";
    }
}

const char* toString(ControlMode m)
{
    switch (m)
    {
        case ControlMode::OPEN_LOOP:
            return "OPEN_LOOP";
        case ControlMode::HYBRID:
            return "HYBRID";
        default:
            return "UNKNOWN";
    }
}
const char* toString(Direction m)
{
    switch (m)
    {
        case Direction::CLOCKWISE:
            return "CW";
        case Direction::COUNTER_CLOCKWISE:
            return "CCW";
        default:
            return "UNK";
    }
}
// Build preference key for a motor
String makeKey(int motorIndex, const char* suffix)
{
    return "m" + String(motorIndex) + "_" + suffix;
}

String isSave(size_t result)
{
    return (result == 0) ? "Failed ❌" : "OK ✅";
}

// Optional: small helper to clamp/validate an int to ControlMode
inline ControlMode safeModeFromInt(int v, ControlMode fallback)
{
    if (v < static_cast<int>(ControlMode::OPEN_LOOP) || v > static_cast<int>(ControlMode::HYBRID))
    {
        return fallback;
    }
    return static_cast<ControlMode>(v);
}

bool isMoving()
{
    return positionController[currentIndex].isMoving();
}
// Pretty-print a fixed-width ASCII table of conversions
static void printConversionTable(float inputValue, const ConvertValues& cvfd, const ConvertValues& cvfp, const ConvertValues& cvfs, const ConvertValues& cvfm)
{
    Serial.println(inputValue);
    auto line = []()
    {
        Serial.println("+----------------------+------------+------------+------------+-------"
                       "-----+--------------+");
    };
    line();
    Serial.printf("| %-20s | %10s | %10s | %10s | %10s | %12s |\n", "From (unit)", "Degrees", "Pulses", "Steps", "Turns", "Micrometers");
    line();

    // From Degrees (inputValue interpreted as degrees)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Degrees", cvfd.TO_DEGREES, static_cast<double>(cvfd.TO_PULSES), static_cast<long>(cvfd.TO_STEPS), static_cast<long>(cvfd.TO_TURNS), static_cast<double>(cvfd.TO_MICROMETERS));

    // From Pulses (inputValue interpreted as pulses)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Pulses", static_cast<double>(cvfp.TO_DEGREES), static_cast<double>(cvfp.TO_PULSES), static_cast<long>(cvfp.TO_STEPS), static_cast<long>(cvfp.TO_TURNS), static_cast<double>(cvfp.TO_MICROMETERS));

    // From Steps (inputValue interpreted as steps)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Steps", static_cast<double>(cvfs.TO_DEGREES), static_cast<double>(cvfs.TO_PULSES), static_cast<long>(cvfs.TO_STEPS), static_cast<long>(cvfs.TO_TURNS), static_cast<double>(cvfs.TO_MICROMETERS));

    // From Micrometers (inputValue interpreted as µm)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Micrometers", static_cast<double>(cvfm.TO_DEGREES), static_cast<double>(cvfm.TO_PULSES), static_cast<long>(cvfm.TO_STEPS), static_cast<long>(cvfm.TO_TURNS), static_cast<double>(cvfm.TO_MICROMETERS));

    line();
}

#pragma endregion

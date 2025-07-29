#include "CommandHistory.h"
#include "DirMultiplexer.h"
#include "MAE3Encoder.h"
#include "PositionController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include "VoltageMonitor.h"
#include "esp_log.h"
#include <Arduino.h>
#include <CLIManager.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_task_wdt.h>

CommandHistory<16> history;
const char*        commands[] = {"Q", "L", "K"};
enum class CommandKey
{
    Q = 0,
    L = 1,
    K = 2,
};

struct Data
{
    struct Position
    {
        ControlMode mode           = ControlMode::OPEN_LOOP;
        float       reached        = 0.0f;
        float       beforeMovement = 0.0f;
        float       value          = 0.0f;
        bool        save           = false;
    } position, voltageDropPosition, orgin, control;
} data;

VoltageMonitor voltageMonitor(VoltageMonitorPins::POWER_3_3, VoltageMonitor::MonitorMode::DIGITAL_, 0, 10);

// Driver status tracking
static bool    driverEnabled[4] = {false, false, false, false};
TMC5160Manager driver[4]        = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]),
                                   TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

// Direction multiplexer for stepper motor direction control
DirMultiplexer dirMultiplexer(MultiplexerPins::S0, MultiplexerPins::S1, MultiplexerPins::DIR);

// MAE3 Encoders for position feedback (read-only, not used for control)
MAE3Encoder encoder[4] = {MAE3Encoder(EncoderPins::SIGNAL[0], 0), MAE3Encoder(EncoderPins::SIGNAL[1], 1),
                          MAE3Encoder(EncoderPins::SIGNAL[2], 2), MAE3Encoder(EncoderPins::SIGNAL[3], 3)};

// Position controllers for precise angle control
PositionController positionController[4] = {
    PositionController(0, driver[0], dirMultiplexer, DriverPins::STEP[0], DriverPins::EN[0], encoder[0]),
    PositionController(1, driver[1], dirMultiplexer, DriverPins::STEP[1], DriverPins::EN[1], encoder[1]),
    PositionController(2, driver[2], dirMultiplexer, DriverPins::STEP[2], DriverPins::EN[2], encoder[2]),
    PositionController(3, driver[3], dirMultiplexer, DriverPins::STEP[3], DriverPins::EN[3], encoder[3])};

static constexpr uint32_t SPI_CLOCK            = 1000000;  // 1MHz SPI clock
TaskHandle_t              serialReadTaskHandle = NULL;

static uint8_t currentIndex = 1;  // Current driver index

// K key press timing
static uint32_t lastKPressTime = 0;  // Timestamp of last K key press

// Current control parameters
static constexpr uint16_t MIN_RMS_CURRENT = 100;  // Minimum RMS current in mA
static constexpr uint16_t MAX_RMS_CURRENT = 500;  // Maximum RMS current in mA

static constexpr uint8_t MIN_IRUN = 1;   // Minimum IRUN value (1/32 of RMS current)
static constexpr uint8_t MAX_IRUN = 31;  // Maximum IRUN value (31/32 of RMS current)

static constexpr uint8_t MIN_IHOLD = 1;   // Minimum IHOLD value (1/32 of RMS current)
static constexpr uint8_t MAX_IHOLD = 31;  // Maximum IHOLD value (31/32 of RMS current)

static constexpr uint16_t CURRENT_INCREMENT = 10;  // Current change increment
static constexpr uint8_t  IRUN_INCREMENT    = 1;   // IRUN change increment
static constexpr uint8_t  IHOLD_INCREMENT   = 1;   // IHOLD change increment

bool voltageMonitorFirstTime = false;

Preferences prefs;

// Function declarations
uint16_t calculateByNumber(uint16_t rms_current, uint8_t number);
void     storeToMemory();
float    loadOriginPosition();
void     loadControlMode();
void     loadPosition();
void     clearAllOriginPositions();
void     printAllOriginPositions();
void     clearLine();
void     setMotorId(String motorId);
void     serialReadTask(void* pvParameters);
float    setCurrentPositionFromEncoder();
void     checkDifferenceCorrection();
void     onVoltageDrop();

void setup()
{
    // Initialize SPI
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(115200);

    esp_log_level_set("*", ESP_LOG_INFO);

    // Initialize Preferences for storing motor origin positions
    if (!prefs.begin("motion_control", false))  // false = read/write mode
    {
        log_e("Failed to initialize Preferences");
    }
    else
    {
        log_d("Preferences successfully initialized");
    }

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
            encoder[index].begin();
            encoder[index].disable();  // Enable encoder for reading
        }
    }

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
    encoder[currentIndex].handleMovementComplete();
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

uint16_t calculateByNumber(uint16_t rms_current, uint8_t number)
{
    return (rms_current * number) / 32;
}

void storeToMemory()
{
    noInterrupts();
    if (data.orgin.save)
    {
        data.orgin.save = false;
        String key      = "m" + String(currentIndex) + "_or";
        log_d("Key: %s, Value: %f", key.c_str(), data.orgin.value);
        bool success = prefs.putFloat(key.c_str(), data.orgin.value);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!success)
        {
            log_e("Failed to store origin position for motor %d", currentIndex + 1);
        }
    }

    if (data.control.save)
    {
        data.control.save = false;
        String key        = "m" + String(currentIndex) + "_cm";
        log_d("Key: %s, Value: %d", key.c_str(), static_cast<int>(data.control.mode));
        bool success = prefs.putInt(key.c_str(), static_cast<int>(data.control.mode));
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!success)
        {
            log_e("Failed to store control mode for motor %d", currentIndex + 1);
        }
    }

    if (data.voltageDropPosition.save)
    {
        data.voltageDropPosition.save = false;
        String key                    = "m" + String(currentIndex) + "_po";
        log_d("Key: %s, Value: %f", key.c_str(), data.voltageDropPosition.beforeMovement);
        bool success = prefs.putFloat(key.c_str(), data.voltageDropPosition.beforeMovement);

        if (!success)
        {
            log_e("Failed to store voltage drop for motor %d", currentIndex + 1);
        }
    }

    if (data.position.save)
    {
        data.position.save = false;
        String key         = "m" + String(currentIndex) + "_po";
        log_d("Key: %s, Value: %f", key.c_str(), data.position.reached);
        bool success = prefs.putFloat(key.c_str(), data.position.reached);

        if (!success)
        {
            log_e("Failed to store target reached for motor %d", currentIndex + 1);
        }
    }
    interrupts();
    encoder[currentIndex].setStorageCompleteFlag(false);
}

float loadOriginPosition()
{
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return 0.0f;
    }

    String key            = "m" + String(currentIndex) + "_or";
    float  originPosition = prefs.getFloat(key.c_str(), 0.0f);  // Default to 0.0 if not found
    log_d("Motor %d origin position loaded: %f°", currentIndex + 1, originPosition);
    return originPosition;
}

void loadControlMode()
{
    String key        = "m" + String(currentIndex) + "_cm";
    data.control.mode = static_cast<ControlMode>(prefs.getInt(key.c_str(), static_cast<int>(ControlMode::OPEN_LOOP)));
    log_d("Motor %d control mode loaded: %d", currentIndex + 1, static_cast<int>(data.control.mode));
}

void loadPosition()
{
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return;
    }

    String key          = "m" + String(currentIndex) + "_po";
    data.position.value = prefs.getFloat(key.c_str(), 0.0f);
    log_d("Motor %d voltage drop loaded: %f", currentIndex + 1, data.position.value);
}

void clearAllOriginPositions()
{
    // Clear origin positions for all motors
    for (uint8_t i = 0; i < 4; i++)
    {
        String key = "m" + String(i) + "_or";
        prefs.remove(key.c_str());
    }

    log_d("All motor origin positions cleared");
}

void printAllOriginPositions()
{
    log_d("Current origin positions:");

    // Add delay to ensure system stability
    delay(10);

    for (uint8_t i = 0; i < 4; i++)
    {
        float originPosition = 0.0f;

        String key = "m" + String(i) + "_or";

        try
        {
            originPosition = prefs.getFloat(key.c_str(), 0.0f);
            log_d("Motor %d: %f°", i + 1, originPosition);
        }
        catch (...)
        {
            log_e("Failed to read origin position for motor %d", i + 1);
            originPosition = 0.0f;
        }

        // Small delay between each motor to prevent overload
        delay(5);
    }

    // Print total number of keys for debugging
    try
    {
        size_t totalKeys = prefs.freeEntries();
        log_d("Total free entries: %d", totalKeys);
    }
    catch (...)
    {
        log_e("Failed to get preferences free entries");
    }
}

void clearLine()
{
    Serial.print(F("\r"));  // Go to the beginning of the line
    Serial.print(
        F("                                                                                "));  // About 80 characters
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
            encoder[index].disable();  // Enable encoder for reading
        }
    }

    encoder[currentIndex].enable();
    log_i("Motor %d selected and enabled", currentIndex + 1);
}

// Serial read task (M101)
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
            /*
            else if (c == commands[static_cast<int>(CommandKey::A)][0])  // 'A' Increase RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms < MAX_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms + CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                }

            }
            else if (c == commands[static_cast<int>(CommandKey::Z)][0])  // 'Z' Decrease RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms > MIN_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms - CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                }

            }
            else if (c == commands[static_cast<int>(CommandKey::S)][0])  // 'S' Increase IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun < MAX_IRUN)
                {
                    uint8_t newIrun = currentIrun + IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                }

            }
            else if (c == commands[static_cast<int>(CommandKey::X)][0])  // 'X' Decrease IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun > MIN_IRUN)
                {
                    uint8_t newIrun = currentIrun - IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                }

            }
            else if (c == commands[static_cast<int>(CommandKey::D)][0])  // 'D' Increase IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold < MAX_IHOLD)
                {
                    uint8_t newIhold = currentIhold + IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                }

            }
            else if (c == commands[static_cast<int>(CommandKey::C)][0])  // 'C' Decrease IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold > MIN_IHOLD)
                {
                    uint8_t newIhold = currentIhold - IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                }

            }
            */
            else if (c == commands[static_cast<int>(CommandKey::Q)][0])  // 'Q' Show position status
            {
                float targetAngle = data.position.value;
                loadControlMode();

                encoder[currentIndex].attachOnComplete(storeToMemory);
                positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);

                bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE,
                                                                            data.control.mode);

                if (success)
                {
                    const char* modeStr = (data.control.mode == ControlMode::OPEN_LOOP)     ? "open-loop"
                                          : (data.control.mode == ControlMode::CLOSED_LOOP) ? "closed-loop"
                                                                                            : "hybrid";
                    log_i("Motor %d moving to %f degrees (%s)", currentIndex + 1, targetAngle, modeStr);
                }
                else
                {
                    log_e("Failed to queue movement command");
                }
            }
            else if (c == commands[static_cast<int>(CommandKey::L)][0])  // 'L' Show position status
            {
                encoder[currentIndex].processPWM();
                EncoderState encoderState = encoder[currentIndex].getState();
                MotorStatus  status       = positionController[currentIndex].getStatus();
                if (status.currentAngle == 0)
                {
                    setCurrentPositionFromEncoder();
                    status.currentAngle = positionController[currentIndex].getCurrentAngle();
                }
                float diff = status.currentAngle - encoderState.position_degrees;
                Serial.print(F("[Position Status] Motor "));
                Serial.print(currentIndex + 1);
                Serial.print(F(": Diff="));
                Serial.print(diff);
                Serial.print(F("°, Current="));
                Serial.print(status.currentAngle);
                Serial.print(F("° 💡, Target="));
                Serial.print(status.targetAngle);
                Serial.print(F("°, Moving="));
                Serial.print(status.isMoving ? F("YES") : F("NO"));
                Serial.print(F(", Enabled="));
                Serial.print(status.isEnabled ? F("YES") : F("NO"));
                Serial.print(F(", Mode="));
                const char* modeStr = (status.controlMode == ControlMode::OPEN_LOOP)     ? "OPEN L"
                                      : (status.controlMode == ControlMode::CLOSED_LOOP) ? "CLOSED L"
                                                                                         : "HYBRID";
                Serial.print(modeStr);
                if (status.controlMode == ControlMode::CLOSED_LOOP)
                {
                    Serial.print(F(", Error="));
                    Serial.print(status.positionError, 2);
                    Serial.print(F("°"));
                }
                Serial.print(F(", (Encoder: "));
                Serial.print(encoderState.position_degrees);
                Serial.print(F("°,"));
                Serial.print(encoderState.direction == Direction::CLOCKWISE ? F(" CW") : F(" CCW"));
                Serial.print(F(", "));
                Serial.print(encoderState.position_pulse);
                Serial.println(F(" pulses)"));
            }
            else if (c == commands[static_cast<int>(CommandKey::K)][0])  // 'K' Show encoder interrupt counters
            {
                uint32_t currentTime  = millis();
                uint32_t timeInterval = 0;

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
                    // Get interrupt counters
                    uint32_t totalInterrupts = encoder[currentIndex].getInterruptCount();
                    uint32_t highEdges       = encoder[currentIndex].getHighEdgeCount();
                    uint32_t lowEdges        = encoder[currentIndex].getLowEdgeCount();

                    Serial.print(F("  Total Interrupts: "));
                    Serial.println(totalInterrupts);
                    Serial.print(F("  Rising Edges (High): "));
                    Serial.println(highEdges);
                    Serial.print(F("  Falling Edges (Low): "));
                    Serial.println(lowEdges);
                    Serial.print(F("  Encoder Enabled: "));
                    Serial.println(encoder[currentIndex].isEnabled() ? F("YES") : F("NO"));
                    if (encoder[currentIndex].isEnabled())
                    {
                        encoder[currentIndex].processPWM();
                        EncoderState encoderState = encoder[currentIndex].getState();
                        Serial.print(F("  Encoder Position: "));
                        Serial.print(encoderState.position_degrees, 2);
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

                    // Reset counters after displaying
                    encoder[currentIndex].resetInterruptCounters();
                    Serial.println(F("  [Counters Reset]"));

                    // Update last K press time
                    lastKPressTime = currentTime;
                }
            }
            /*
            else if (c == commands[static_cast<int>(CommandKey::T)][0])  // 'T' Test distance-based speed control
            {
                Serial.println();
                Serial.println(F("[Distance-Based Speed Control Test]"));
                Serial.println(F("Testing different movement distances:"));

                // Test distances: 0.2°, 0.4°, 0.5°, 1°, 5°, 10°, 50°
                float       testDistances[] = {0.2f, 0.4f, 0.5f, 1.0f, 5.0f, 10.0f, 50.0f};
                const char* distanceNames[] = {"0.2°", "0.4°", "0.5°", "1°  ", "5°  ", "10° ", "50° "};

                for (int i = 0; i < 7; i++)
                {
                    float        distance     = testDistances[i];
                    DistanceType distanceType = positionController[currentIndex].calculateDistanceType(distance);
                    float        optimalSpeed =
            positionController[currentIndex].calculateOptimalSpeedForDistance(distance); float        optimalAccel =
            positionController[currentIndex].calculateOptimalAccelerationForDistance(distance);

                    const char* typeStr = "";
                    switch (distanceType)
                    {
                        case DistanceType::VERY_SHORT:
                            typeStr = "VERY_SHORT";
                            break;
                        case DistanceType::SHORT:
                            typeStr = "SHORT";
                            break;
                        case DistanceType::MEDIUM:
                            typeStr = "MEDIUM";
                            break;
                        case DistanceType::LONG:
                            typeStr = "LONG";
                            break;
                        case DistanceType::VERY_LONG:
                            typeStr = "VERY_LONG";
                            break;
                        default:
                            typeStr = "NEGLIGIBLE";
                            break;
                    }

                    Serial.printf("  %s: Type=%s, Speed=%.0f steps/s, Accel=%.0f steps/s²\n", distanceNames[i], typeStr,
            optimalSpeed, optimalAccel);
                }

                Serial.println(F("Press '1'-'7' to test these movements, or any other key to continue"));
            }
            else if (c >= '1' && c <= '7')  // Test specific distance movements
            {
                float testDistances[] = {0.2f, 0.4f, 0.5f, 1.0f, 5.0f, 10.0f, 50.0f};
                int   index           = c - '1';
                float distance        = testDistances[index];

                float currentAngle = positionController[currentIndex].getCurrentAngle();
                float targetAngle  = PositionController::wrapAngle(currentAngle + distance);

                log_i("Testing %f° movement (current: %.2f° -> target: %.2f°)\n", distance, currentAngle, targetAngle);

                bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE,
            ControlMode::OPEN_LOOP);

                if (success)
                {
                    log_i("Motor %d testing %f° movement", currentIndex + 1, distance);
                }
                else
                {
                    log_e("Failed to queue test movement");
                }
            }
            */
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

                    if (0)
                    {
                        if (motorIdStr == "1")
                        {
                            currentIndex = 0;
                            positionController[currentIndex].setDirection(true);
                        }
                        else if (motorIdStr == "2")
                        {
                            currentIndex = 1;
                            positionController[currentIndex].setDirection(true);
                        }
                        else if (motorIdStr == "3")
                        {
                            currentIndex = 2;
                            positionController[currentIndex].setDirection(true);
                        }
                        else if (motorIdStr == "4")
                        {
                            currentIndex = 3;
                            positionController[currentIndex].setDirection(true);
                        }
                        else if (motorIdStr == "5")
                        {
                            currentIndex = 0;
                            positionController[currentIndex].setDirection(false);
                        }
                        else if (motorIdStr == "6")
                        {
                            currentIndex = 1;
                            positionController[currentIndex].setDirection(false);
                        }
                        else if (motorIdStr == "7")
                        {
                            currentIndex = 2;
                            positionController[currentIndex].setDirection(false);
                        }
                        else if (motorIdStr == "8")
                        {
                            currentIndex = 3;
                            positionController[currentIndex].setDirection(false);
                        }
                    }
                    setMotorId(motorIdStr);
                }
            }
            else if (c == cmdMove)
            {
                if (c.getArgument("p").isSet())
                {
                    String degreesStr  = c.getArgument("p").getValue();
                    float  targetAngle = degreesStr.toFloat();
                    if (targetAngle == 0)
                        targetAngle = 0.01;
                    else if (targetAngle == 360)
                        targetAngle = 359.9955f;

                    data.voltageDropPosition.beforeMovement = setCurrentPositionFromEncoder();
                    loadControlMode();

                    encoder[currentIndex].attachOnComplete(storeToMemory);
                    positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);

                    bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE,
                                                                                data.control.mode);

                    if (success)
                    {
                        const char* modeStr = (data.control.mode == ControlMode::OPEN_LOOP)     ? "open-loop"
                                              : (data.control.mode == ControlMode::CLOSED_LOOP) ? "closed-loop"
                                                                                                : "hybrid";
                        log_i("Motor %d moving to %f degrees (%s)", currentIndex + 1, targetAngle, modeStr);
                    }
                    else
                    {
                        log_e("Failed to queue movement command");
                    }
                }
            }
            else if (c == cmdMoveRelative)
            {
                if (c.getArgument("p").isSet())
                {
                    String degreesStr  = c.getArgument("p").getValue();
                    float  targetAngle = degreesStr.toFloat();
                    log_i("Motor %d moving relative to %f degrees", currentIndex + 1, targetAngle);
                }
            }
            else if (c == cmdControlMode)
            {
                if (c.getArgument("c").isSet())
                {
                    data.control.mode = ControlMode::CLOSED_LOOP;
                }
                else if (c.getArgument("o").isSet())
                {
                    data.control.mode = ControlMode::OPEN_LOOP;
                }
                else if (c.getArgument("h").isSet())
                {
                    data.control.mode = ControlMode::HYBRID;
                }

                data.control.save = true;
                storeToMemory();
            }
            else if (c == cmdEnable)
            {
                positionController[currentIndex].enable();
            }
            else if (c == cmdDisable)
            {
                positionController[currentIndex].stop();
                vTaskDelay(pdMS_TO_TICKS(100));
                positionController[currentIndex].disable();
            }
            else if (c == cmdCurrentPosition)
            {
                float position = positionController[currentIndex].getCurrentAngle();
                Serial.print(F("*"));
                Serial.print(currentIndex);
                Serial.print(F("#"));
                Serial.print(position, 2);
                Serial.println(F("#"));
            }
            else if (c == cmdSave)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);

                    if (!encoder[currentIndex].isEnabled())
                    {
                        log_w("Encoder not enabled");
                    }
                    else
                    {
                        float value      = setCurrentPositionFromEncoder();
                        data.orgin.value = value;
                        data.orgin.save  = true;

                        if (1)
                        {
                            ConvertValuesFromDegrees cvfd = positionController[currentIndex].convertFromDegrees(value);
                            ConvertValuesFromPulses  cvfp = positionController[currentIndex].convertFromPulses(value);
                            ConvertValuesFromSteps   cvfs = positionController[currentIndex].convertFromMSteps(value);
                            Serial.print(F("From Degrees: "));
                            Serial.print(cvfd.PULSES_FROM_DEGREES);
                            Serial.print(F(", "));
                            Serial.println(cvfd.STEPS_FROM_DEGREES);
                            Serial.print(F("From Pulses: "));
                            Serial.print(cvfp.DEGREES_FROM_PULSES);
                            Serial.print(F(", "));
                            Serial.println(cvfp.STEPS_FROM_PULSES);
                            Serial.print(F("From Steps: "));
                            Serial.print(cvfs.DEGREES_FROM_STEPS);
                            Serial.print(F(", "));
                            Serial.println(cvfs.PULSES_FROM_STEPS);
                        }
                    }
                }
            }
            else if (c == cmdRestart)
            {
                ESP.restart();
            }
            else if (c == cmdStop)
            {
                positionController[currentIndex].stop();
            }
            else if (c == cmdShow)
            {
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.print(cli.toString());
                Serial.println();
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

float setCurrentPositionFromEncoder()  // amir
{
    float   encoderAngle = positionController[currentIndex].getEncoderAngle();
    int32_t steps        = positionController[currentIndex].convertFromDegrees(encoderAngle).STEPS_FROM_DEGREES;
    positionController[currentIndex].setCurrentPosition(steps);
    log_d("Motor %d current position set to Angle: %f", currentIndex + 1, encoderAngle);
    return encoderAngle;
}

void checkDifferenceCorrection()
{
    encoder[currentIndex].processPWM();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_task_wdt_reset();
    float encoderAngle = positionController[currentIndex].getEncoderAngle();
    float currentAngle = positionController[currentIndex].getCurrentAngle();
    float targetAngle  = positionController[currentIndex].getTargetAngle();
    //  Serial.printf("Encoder: %f, Current: %f, Target: %f\n", encoderAngle, currentAngle, targetAngle);

    float difference = encoderAngle - currentAngle;

    if (data.control.mode == ControlMode::OPEN_LOOP && fabs(difference) > 0.05)
    {
        int32_t steps = positionController[currentIndex].convertFromDegrees(encoderAngle).STEPS_FROM_DEGREES;
        positionController[currentIndex].setCurrentPosition(steps);

        if (targetAngle == 0)
            targetAngle = 0.01;
        else if (targetAngle == 360)
            targetAngle = 359.9955f;

        // encoder[currentIndex].attachOnComplete(storeOriginPosition);
        positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);

        bool success =
            positionController[currentIndex].moveToAngle(targetAngle, MovementType::SHORT_RANGE, data.control.mode);

        if (success)
        {
            log_d("Motor %d moving to %f degrees (%s)", currentIndex + 1, targetAngle, "open-loop");
        }
        else
        {
            log_e("Failed to queue movement command");
        }
    }
    else
    {
        data.position.reached = currentAngle;
        data.position.save    = true;
        storeToMemory();
        log_i("No movement needed, difference: %f, ControlMode: %s", difference,
              (data.control.mode == ControlMode::OPEN_LOOP)     ? "open-loop"
              : (data.control.mode == ControlMode::CLOSED_LOOP) ? "closed-loop"
                                                                : "hybrid");
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

    if (positionController[currentIndex].isMoving())
    {
        positionController[currentIndex].stop();
        data.voltageDropPosition.save = true;
        storeToMemory();
    }
}
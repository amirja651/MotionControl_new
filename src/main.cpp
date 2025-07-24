#include "MAE3Encoder.h"
#include "PositionController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <CLIManager.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_task_wdt.h>

struct OrginData
{
    uint8_t motorIndex = 0;
    float   value      = 0.0f;
    bool    save       = false;
} orgin;

// Driver status tracking
static bool    driverEnabled[4] = {false, false, false, false};
TMC5160Manager driver[4]        = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

// MAE3 Encoders for position feedback (read-only, not used for control)
MAE3Encoder encoder[4] = {MAE3Encoder(EncoderPins::SIGNAL[0], 0), MAE3Encoder(EncoderPins::SIGNAL[1], 1), MAE3Encoder(EncoderPins::SIGNAL[2], 2), MAE3Encoder(EncoderPins::SIGNAL[3], 3)};

// Position controllers for precise angle control
PositionController positionController[4] = {PositionController(0, driver[0], DriverPins::DIR[0], DriverPins::STEP[0], DriverPins::EN[0], encoder[0]), PositionController(1, driver[1], DriverPins::DIR[1], DriverPins::STEP[1], DriverPins::EN[1], encoder[1]),
                                            PositionController(2, driver[2], DriverPins::DIR[2], DriverPins::STEP[2], DriverPins::EN[2], encoder[2]), PositionController(3, driver[3], DriverPins::DIR[3], DriverPins::STEP[3], DriverPins::EN[3], encoder[3])};

static constexpr uint32_t SPI_CLOCK            = 1000000;  // 1MHz SPI clock
TaskHandle_t              serialReadTaskHandle = NULL;

// Command history support
#define HISTORY_SIZE 10
static String  commandHistory[HISTORY_SIZE];
static int     historyCount = 0;
static int     historyIndex = -1;  // -1 means not navigating
static uint8_t currentIndex = 1;   // Current driver index

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

Preferences prefs;

// Function declarations
uint16_t calculateByNumber(uint16_t rms_current, uint8_t number);
void     storeOriginPosition();
float    loadOriginPosition(uint8_t motorIndex);
void     clearAllOriginPositions();
void     printAllOriginPositions();
void     printCurrentSettingsAndKeyboardControls();
void     clearLine();
void     setMotorId(String motorId);
void     serialReadTask(void* pvParameters);

void setup()
{
    // Initialize SPI
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(115200);

    // Initialize Preferences for storing motor origin positions
    if (!prefs.begin("motion_control", false))  // false = read/write mode
    {
        log_e("Failed to initialize Preferences");
    }
    else
    {
        log_i("Preferences successfully initialized");
    }

    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

#ifdef CONFIG_FREERTOS_CHECK_STACKOVERFLOW
    log_i("Stack overflow checking enabled");
#else
    log_i("Stack overflow checking **NOT** enabled");
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
        // Validate pin number before using
        if (DriverPins::CS[index] < 40)  // ESP32 has max 40 GPIO pins
        {
            pinMode(DriverPins::CS[index], OUTPUT);
            digitalWrite(DriverPins::CS[index], HIGH);
        }
        else
        {
            log_e("Invalid CS pin number: %d", DriverPins::CS[index]);
        }
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

            // Configure motor parameters
            driver[index].configureDriver_All_Motors(true);
        }
    }

    // Initialize position controllers and encoders
    for (uint8_t index = 0; index < 4; index++)
    {
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
    delay(500);

    try
    {
        printAllOriginPositions();
    }
    catch (...)
    {
        log_e("Failed to print origin positions, continuing...");
    }

    delay(1000);

    esp_task_wdt_init(15, true);  // Increased timeout to 15 seconds
    esp_task_wdt_add(NULL);       // Add the current task (setup)
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    // Create serial read task with larger stack
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 8192, NULL, 2, &serialReadTaskHandle, 0);
    esp_task_wdt_add(serialReadTaskHandle);  // Register with WDT

    log_i("Position control system initialized");
    log_i("Use 'L' to show position status");
}

void loop()
{
    // Handle movement complete outside ISR
    encoder[currentIndex].handleMovementComplete();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
}

uint16_t calculateByNumber(uint16_t rms_current, uint8_t number)
{
    return (rms_current * number) / 32;
}

void storeOriginPosition()
{
    if (!orgin.save)
    {
        encoder[currentIndex].setStorageCompleteFlag(false);
        return;
    }

    if (orgin.motorIndex >= 4)
    {
        log_e("Invalid motor index");
        return;
    }

    orgin.save = false;

    // Create a unique key for each motor's origin position
    String key = "motor" + String(orgin.motorIndex) + "_origin";
    log_i("Key: %s, Value: %f", key.c_str(), orgin.value);
    noInterrupts();
    bool success = prefs.putFloat(key.c_str(), orgin.value);
    interrupts();
    if (!success)
    {
        log_e("Failed to store origin position for motor %d", orgin.motorIndex + 1);
    }

    encoder[currentIndex].setStorageCompleteFlag(false);
}

float loadOriginPosition(uint8_t motorIndex)
{
    if (motorIndex >= 4)
    {
        log_e("Invalid motor index, using 0.0");
        return 0.0f;
    }

    // Create a unique key for each motor's origin position
    String key = "motor" + String(motorIndex) + "_origin";

    float originPosition = 0.0f;
    try
    {
        originPosition = prefs.getFloat(key.c_str(), 0.0f);  // Default to 0.0 if not found
    }
    catch (...)
    {
        log_e("Exception occurred while loading origin position for motor %d", motorIndex + 1);
        return 0.0f;
    }

    log_i("Motor %d origin position loaded: %f°", motorIndex + 1, originPosition);

    return originPosition;
}

void clearAllOriginPositions()
{
    // Clear origin positions for all motors
    for (uint8_t i = 0; i < 4; i++)
    {
        String key = "motor" + String(i) + "_origin";
        prefs.remove(key.c_str());
    }

    log_i("All motor origin positions cleared");
}

void printAllOriginPositions()
{
    log_i("Current origin positions:");

    // Add delay to ensure system stability
    delay(10);

    for (uint8_t i = 0; i < 4; i++)
    {
        float originPosition = 0.0f;

        String key = "motor" + String(i) + "_origin";

        try
        {
            originPosition = prefs.getFloat(key.c_str(), 0.0f);
            log_i("Motor %d: %f°", i + 1, originPosition);
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
        log_i("Total free entries: %d", totalKeys);
    }
    catch (...)
    {
        log_e("Failed to get preferences free entries");
    }
}

void printCurrentSettingsAndKeyboardControls()
{
    Serial.println(F("========================= Keyboard Controls ============================"));

    Serial.print(F("  RMS Current:      'A' (+) | 'Z' (-)    "));
    Serial.print(driver[currentIndex].getRmsCurrent());
    Serial.println(F(" mA"));

    Serial.print(F("  IRUN Current:     'S' (+) | 'X' (-)    "));
    Serial.print(driver[currentIndex].getIrun());
    Serial.print(F("/32 of RMS current"));
    Serial.print(F(" ("));
    Serial.print(calculateByNumber(driver[currentIndex].getRmsCurrent(), driver[currentIndex].getIrun()));
    Serial.println(F(" mA)"));

    Serial.print(F("  IHOLD Current:    'D' (+) | 'C' (-)    "));
    Serial.print(driver[currentIndex].getIhold());
    Serial.print(F("/32 of RMS current"));
    Serial.print(F(" ("));
    Serial.print(calculateByNumber(driver[currentIndex].getRmsCurrent(), driver[currentIndex].getIhold()));
    Serial.println(F(" mA)"));

    Serial.print(F("  Microsteps: "));
    Serial.println(driver[currentIndex].getMicrosteps());

    Serial.println(F("  Position Control: 'Q' Move to origin (open-loop)"));
    Serial.println(F("  Position Control: 'J' Move to origin (closed-loop)"));
    Serial.println(F("  Position Status:  'L' Show position status"));
    Serial.println(F("  Encoder Counters: 'K' Show & reset interrupt counters"));
    Serial.println(F("  Distance Test:    'T' Test distance-based speed control"));
    Serial.println(F("  Test Movements:   '1'-'7' Test specific distances (0.2°-50°)"));
    Serial.println(F(""));
    Serial.println(F("Distance-Based Speed Control:"));
    Serial.println(F("  < 0.1°: Ignored (negligible)"));
    Serial.println(F("  0.1°-0.5°: Very slow & precise"));
    Serial.println(F("  0.5°-1°: Slow & precise"));
    Serial.println(F("  1°-5°: Balanced speed/precision"));
    Serial.println(F("  5°-10°: Moderate speed"));
    Serial.println(F("  > 10°: Higher speed"));
    Serial.println(F("========================================================================"));
}

void clearLine()
{
    Serial.print(F("\r"));                                                                                // Go to the beginning of the line
    Serial.print(F("                                                                                "));  // About 80 characters
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
                    if (historyCount > 0)
                    {
                        if (historyIndex < historyCount - 1)
                            historyIndex++;
                        inputBuffer = commandHistory[(historyCount - 1 - historyIndex) % HISTORY_SIZE];
                        clearLine();
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
                if (c == 'B')
                {  // Down arrow
                    if (historyCount > 0 && historyIndex > 0)
                    {
                        historyIndex--;
                        inputBuffer = commandHistory[(historyCount - 1 - historyIndex) % HISTORY_SIZE];
                        clearLine();
                        Serial.print(inputBuffer);
                    }
                    else if (historyIndex == 0 && isPrintable(c))
                    {
                        historyIndex = -1;
                        inputBuffer  = lastInput;
                        clearLine();
                        Serial.print(F("> "));
                        Serial.print(inputBuffer);
                    }
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

            const char* commands[] = {"A", "Z", "S", "X", "D", "C", "Q", "J", "L", "K", "T"};
            // Handle Enter
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.println();
                    Serial.print(F("# "));
                    Serial.println(inputBuffer.c_str());
                    cli.parse(inputBuffer);

                    // Add to history
                    if (historyCount == 0 || commandHistory[(historyCount - 1) % HISTORY_SIZE] != inputBuffer)
                    {
                        commandHistory[historyCount % HISTORY_SIZE] = inputBuffer;

                        if (historyCount < HISTORY_SIZE)
                            historyCount++;
                    }
                    if (historyIndex > 0)
                        historyIndex = historyCount;
                    else
                        historyIndex = -1;
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
            else if (c == commands[0][0])  // 'A' Increase RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms < MAX_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms + CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                }
                printCurrentSettingsAndKeyboardControls();
            }
            else if (c == commands[1][0])  // 'Z' Decrease RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms > MIN_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms - CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                }
                printCurrentSettingsAndKeyboardControls();
            }
            else if (c == commands[2][0])  // 'S' Increase IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun < MAX_IRUN)
                {
                    uint8_t newIrun = currentIrun + IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                }
                printCurrentSettingsAndKeyboardControls();
            }
            else if (c == commands[3][0])  // 'X' Decrease IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun > MIN_IRUN)
                {
                    uint8_t newIrun = currentIrun - IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                }
                printCurrentSettingsAndKeyboardControls();
            }
            else if (c == commands[4][0])  // 'D' Increase IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold < MAX_IHOLD)
                {
                    uint8_t newIhold = currentIhold + IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                }
                printCurrentSettingsAndKeyboardControls();
            }
            else if (c == commands[5][0])  // 'C' Decrease IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold > MIN_IHOLD)
                {
                    uint8_t newIhold = currentIhold - IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                }
                printCurrentSettingsAndKeyboardControls();
            }
            */
            else if (c == commands[6][0])  // 'Q' Move to 0 degrees
            {
                float targetAngle = loadOriginPosition(currentIndex);

                if (targetAngle == 0)
                    targetAngle = 0.01;
                else if (targetAngle == 360)
                    targetAngle = 359.9955f;

                bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);

                if (success)
                {
                    log_i("Motor %d moving to %f degrees (open-loop)", currentIndex + 1, targetAngle);
                }
                else
                {
                    log_e("Failed to queue movement command");
                }
            }
            else if (c == commands[7][0])  // 'J' Move to 0 degrees with closed-loop
            {
                float targetAngle = loadOriginPosition(currentIndex);

                if (targetAngle == 0)
                    targetAngle = 0.01;
                else if (targetAngle == 360)
                    targetAngle = 359.9955f;

                bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE, ControlMode::CLOSED_LOOP);

                if (success)
                {
                    log_i("Motor %d moving to %f degrees (closed-loop)", currentIndex + 1, targetAngle);
                }
                else
                {
                    log_e("Failed to queue movement command");
                }
            }
            else if (c == commands[8][0])  // 'L' Show position status
            {
                encoder[currentIndex].processPWM();
                EncoderState encoderState = encoder[currentIndex].getState();
                MotorStatus  status       = positionController[currentIndex].getStatus();
                if (status.currentAngle == 0)
                {
                    int32_t steps = positionController[currentIndex].convertFromDegrees(positionController[currentIndex].getEncoderAngle()).STEPS_FROM_DEGREES;
                    positionController[currentIndex].setCurrentPosition(steps);
                    status.currentAngle = positionController[currentIndex].getCurrentAngle();
                }
                float diff = status.currentAngle - encoderState.position_degrees;
                Serial.print(F("[Position Status] Motor "));
                Serial.print(currentIndex + 1);
                Serial.print(F(": Diff="));
                Serial.print(diff);
                Serial.print(F("°, Current="));
                Serial.print(status.currentAngle);
                Serial.print(F("°, Target="));
                Serial.print(status.targetAngle);
                Serial.print(F("°, Moving="));
                Serial.print(status.isMoving ? F("YES") : F("NO"));
                Serial.print(F(", Enabled="));
                Serial.print(status.isEnabled ? F("YES") : F("NO"));
                Serial.print(F(", Mode="));
                const char* modeStr = (status.controlMode == ControlMode::OPEN_LOOP) ? "OPEN L" : (status.controlMode == ControlMode::CLOSED_LOOP) ? "CLOSED L" : "HYBRID";
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
            else if (c == commands[9][0])  // 'K' Show encoder interrupt counters
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
            else if (c == commands[10][0])  // 'T' Test distance-based speed control
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
                    float        optimalSpeed = positionController[currentIndex].calculateOptimalSpeedForDistance(distance);
                    float        optimalAccel = positionController[currentIndex].calculateOptimalAccelerationForDistance(distance);

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

                    Serial.printf("  %s: Type=%s, Speed=%.0f steps/s, Accel=%.0f steps/s²\n", distanceNames[i], typeStr, optimalSpeed, optimalAccel);
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

                bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);

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
                    if (historyIndex == -1)
                        lastInput = inputBuffer;
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
                // Get Motor Id
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                if (c.getArgument("p").isSet())
                {
                    String degreesStr  = c.getArgument("p").getValue();
                    float  targetAngle = degreesStr.toFloat();

                    // Check control mode flags
                    ControlMode controlMode = ControlMode::OPEN_LOOP;
                    if (c.getArgument("j").isSet())
                    {
                        controlMode = ControlMode::CLOSED_LOOP;
                    }
                    else if (c.getArgument("h").isSet())
                    {
                        controlMode = ControlMode::HYBRID;
                    }

                    if (targetAngle == 0)
                        targetAngle = 0.01;
                    else if (targetAngle == 360)
                        targetAngle = 359.9955f;

                    encoder[currentIndex].attachOnComplete(storeOriginPosition);
                    bool success = positionController[currentIndex].moveToAngle(targetAngle, MovementType::MEDIUM_RANGE, controlMode);

                    if (success)
                    {
                        const char* modeStr = (controlMode == ControlMode::OPEN_LOOP) ? "open-loop" : (controlMode == ControlMode::CLOSED_LOOP) ? "closed-loop" : "hybrid";
                        log_i("Motor %d moving to %f degrees (%s)", currentIndex + 1, targetAngle, modeStr);
                    }
                    else
                    {
                        log_e("Failed to queue movement command");
                    }
                }
                else if (c.getArgument("c").isSet())
                {
                    float position = positionController[currentIndex].getCurrentAngle();
                    Serial.print(F("*"));
                    Serial.print(currentIndex);
                    Serial.print(F("#"));
                    Serial.print(position, 2);
                    Serial.println(F("#"));
                }
                else if (c.getArgument("d").isSet())
                {
                    positionController[currentIndex].stop();
                    positionController[currentIndex].disable();
                }
                else if (c.getArgument("e").isSet())
                {
                    positionController[currentIndex].enable();
                }
            }
            else if (c == cmdOrgin)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);

                    if (!encoder[currentIndex].isEnabled())
                    {
                        log_e("Encoder not enabled");
                    }
                    else
                    {
                        // Process encoder to get current reading
                        encoder[currentIndex].processPWM();

                        // Get encoder state
                        EncoderState encoderState = encoder[currentIndex].getState();

                        float value = encoderState.position_degrees;

                        ConvertValuesFromDegrees cvfd = positionController[currentIndex].convertFromDegrees(value);
                        ConvertValuesFromPulses  cvfp = positionController[currentIndex].convertFromPulses(value);
                        ConvertValuesFromSteps   cvfs = positionController[currentIndex].convertFromMSteps(value);

                        if (1)
                        {
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
                        positionController[currentIndex].setCurrentPosition(cvfd.STEPS_FROM_DEGREES);

                        orgin.motorIndex = currentIndex;
                        orgin.value      = value;
                        orgin.save       = true;
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
                printCurrentSettingsAndKeyboardControls();
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

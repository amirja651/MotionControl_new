#include "MAE3Encoder.h"
#include "PositionController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <CLIManager.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_task_wdt.h>

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
void     storeOriginPosition(uint8_t motorIndex, float originDegrees);
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
    // Monitor system health
    static uint32_t lastHealthCheck = 0;
    static uint32_t resetCount      = 0;

    uint32_t currentTime = millis();

    // Health check every 30 seconds
    if (currentTime - lastHealthCheck > 30000)
    {
        lastHealthCheck = currentTime;

        // Check free heap
        uint32_t freeHeap = ESP.getFreeHeap();
        if (freeHeap < 10000)
        {  // Less than 10KB free
            log_w("Low memory: %d bytes free", freeHeap);
        }

        // Check stack usage
        uint32_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
        if (stackHighWater < 1000)
        {  // Less than 1KB stack free
            log_w("Low stack: %d bytes free", stackHighWater);
        }

        // Log system uptime
        log_i("System uptime: %u seconds, Reset count: %u", (unsigned int)(currentTime / 1000), (unsigned int)++resetCount);
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
}

uint16_t calculateByNumber(uint16_t rms_current, uint8_t number)
{
    return (rms_current * number) / 32;
}

void storeOriginPosition(uint8_t motorIndex, float originDegrees)
{
    if (motorIndex >= 4)
    {
        log_e("Invalid motor index");
        return;
    }

    // Validate angle range (0-360 degrees)
    float wrappedAngle = fmod(originDegrees, 360.0f);
    if (wrappedAngle < 0)
        wrappedAngle += 360.0f;

    // Create a unique key for each motor's origin position
    String key = "motor" + String(motorIndex) + "_origin";

    bool success = false;
    try
    {
        success = prefs.putFloat(key.c_str(), wrappedAngle);
    }
    catch (...)
    {
        log_e("Exception occurred while storing origin position for motor %d", motorIndex + 1);
        return;
    }

    if (success)
    {
        if (0)
        {
            log_i("Motor %d origin position stored: %f°", motorIndex + 1, wrappedAngle);
        }
    }
    else
    {
        log_e("Failed to store origin position for motor %d", motorIndex + 1);
    }
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
    Serial.println(F("========================================================================"));
    Serial.flush();
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

    // Task health monitoring
    static uint32_t taskStartTime      = millis();
    static uint32_t watchdogResetCount = 0;

    while (1)
    {
        // Reset watchdog more frequently and track it
        esp_task_wdt_reset();
        watchdogResetCount++;

        // Log task health every 1000 cycles (about 100 seconds)
        if (watchdogResetCount % 1000 == 0)
        {
            uint32_t taskUptime = (millis() - taskStartTime) / 1000;
            uint32_t stackFree  = uxTaskGetStackHighWaterMark(NULL);
            log_i("SerialRead uptime: %u s, Stack: %u bytes, WDT resets: %u", (unsigned int)taskUptime, (unsigned int)stackFree, (unsigned int)watchdogResetCount);
        }

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
                        // Clear current line and print inputBuffer
                        clearLine();
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
                else if (c == 'B')
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

            // Handle Enter
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.println();
                    Serial.print(F("# "));
                    Serial.println(inputBuffer.c_str());

                    try
                    {
                        cli.parse(inputBuffer);
                    }
                    catch (...)
                    {
                        log_e("CLI parsing failed");
                    }

                    // Add to history
                    if (historyCount == 0 || commandHistory[(historyCount - 1) % HISTORY_SIZE] != inputBuffer)
                    {
                        commandHistory[historyCount % HISTORY_SIZE] = inputBuffer;

                        if (historyCount < HISTORY_SIZE)
                            historyCount++;
                    }
                    historyIndex = -1;
                    lastInput    = "";
                    inputBuffer  = "";  // Clear the buffer
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
            else if (c == 'A')  // Increase RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms < MAX_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms + CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                }
                printCurrentSettingsAndKeyboardControls();
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'Z')  // Decrease RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms > MIN_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms - CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                }
                printCurrentSettingsAndKeyboardControls();
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'S')  // Increase IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun < MAX_IRUN)
                {
                    uint8_t newIrun = currentIrun + IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                }
                printCurrentSettingsAndKeyboardControls();
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'X')  // Decrease IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun > MIN_IRUN)
                {
                    uint8_t newIrun = currentIrun - IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                }
                printCurrentSettingsAndKeyboardControls();
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'D')  // Increase IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold < MAX_IHOLD)
                {
                    uint8_t newIhold = currentIhold + IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                }
                printCurrentSettingsAndKeyboardControls();
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'C')  // Decrease IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold > MIN_IHOLD)
                {
                    uint8_t newIhold = currentIhold - IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                }
                printCurrentSettingsAndKeyboardControls();
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'Q')  // Move to 0 degrees
            {
                float targetAngle = loadOriginPosition(currentIndex);
                // Use new position controller
                MovementType movementType = MovementType::MEDIUM_RANGE;
                if (abs(targetAngle) <= 40.0f)
                    movementType = MovementType::SHORT_RANGE;
                else if (abs(targetAngle) >= 100.0f)
                    movementType = MovementType::LONG_RANGE;

                if (targetAngle == 0)
                    targetAngle = 0.01;
                else if (targetAngle == 360)
                    targetAngle = 359.9955f;

                bool success = positionController[currentIndex].moveToAngle(targetAngle, movementType, false);

                if (success)
                {
                    log_i("Motor %d moving to %f degrees (open-loop)", currentIndex + 1, targetAngle);
                }
                else
                {
                    log_e("Failed to queue movement command");
                }

                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'J')  // Move to 0 degrees with closed-loop
            {
                float targetAngle = loadOriginPosition(currentIndex);
                // Use new position controller
                MovementType movementType = MovementType::MEDIUM_RANGE;
                if (abs(targetAngle) <= 40.0f)
                    movementType = MovementType::SHORT_RANGE;
                else if (abs(targetAngle) >= 100.0f)
                    movementType = MovementType::LONG_RANGE;

                if (targetAngle == 0)
                    targetAngle = 0.01;
                else if (targetAngle == 360)
                    targetAngle = 359.9955f;

                bool success = positionController[currentIndex].moveToAngle(targetAngle, movementType, true);

                if (success)
                {
                    log_i("Motor %d moving to %f degrees (closed-loop)", currentIndex + 1, targetAngle);
                }
                else
                {
                    log_e("Failed to queue movement command");
                }

                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'L')  // Show position status
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
                Serial.println();
                Serial.print(F("[Position Status] Motor "));
                Serial.print(currentIndex + 1);
                Serial.print(F(": Current="));
                Serial.print(status.currentAngle);
                Serial.print(F("°, Target="));
                Serial.print(status.targetAngle);
                Serial.print(F("°, Moving="));
                Serial.print(status.isMoving ? F("YES") : F("NO"));
                Serial.print(F(", Enabled="));
                Serial.print(status.isEnabled ? F("YES") : F("NO"));
                Serial.print(F(", Closed-Loop="));
                Serial.print(status.isClosedLoop ? F("YES") : F("NO"));
                if (status.isClosedLoop)
                {
                    Serial.print(F(", Error="));
                    Serial.print(status.positionError, 2);
                    Serial.print(F("°"));
                }
                Serial.print(F(", (Encoder: "));
                Serial.print(encoderState.position_degrees, 2);
                Serial.print(F("°,"));
                Serial.print(encoderState.direction == Direction::CLOCKWISE ? F(" CW") : F(" CCW"));
                Serial.print(F(", "));
                Serial.print(encoderState.position_pulse);
                Serial.print(F(" pulses)"));
                Serial.println();
                inputBuffer = "";
                lastInput   = "";
                Serial.flush();
            }
            else if (c == 'K')  // Show encoder interrupt counters
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
                    inputBuffer = "";
                    lastInput   = "";
                    return;
                }

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

                inputBuffer = "";
                lastInput   = "";
                Serial.flush();
            }
            else if (c != 'A' && c != 'Z' && c != 'S' && c != 'X' && c != 'D' && c != 'C' && c != 'Q' && c != 'L' && c != 'K')  // Only add to buffer if not a direct command
            {
                inputBuffer += c;  // Add character to buffer
                Serial.print(c);
                if (historyIndex == -1)
                    lastInput = inputBuffer;
            }
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }

        if (cli.available())
        {
            Command c = cli.getCmd();

            // Handle motor commands
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

                    // Check if closed-loop flag is set
                    bool closedLoop = c.getArgument("j").isSet();

                    // Use new position controller
                    MovementType movementType = MovementType::MEDIUM_RANGE;
                    if (abs(targetAngle) <= 40.0f)
                        movementType = MovementType::SHORT_RANGE;
                    else if (abs(targetAngle) >= 100.0f)
                        movementType = MovementType::LONG_RANGE;

                    if (targetAngle == 0)
                        targetAngle = 0.01;
                    else if (targetAngle == 360)
                        targetAngle = 359.9955f;

                    bool success = positionController[currentIndex].moveToAngle(targetAngle, movementType, closedLoop);

                    if (success)
                    {
                        log_i("Motor %d moving to %f degrees (%s)", currentIndex + 1, targetAngle, closedLoop ? "closed-loop" : "open-loop");
                    }
                    else
                    {
                        log_e("Failed to queue movement command");
                    }
                }
                else if (c.getArgument("o").isSet())
                {
                    String valueStr = c.getArgument("o").getValue();

                    if (valueStr != "-9999.0")
                    {
                        float                    value = valueStr.toFloat();
                        ConvertValuesFromDegrees cvfd  = positionController[currentIndex].convertFromDegrees(value);
                        ConvertValuesFromPulses  cvfp  = positionController[currentIndex].convertFromPulses(value);
                        ConvertValuesFromSteps   cvfs  = positionController[currentIndex].convertFromMSteps(value);
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

                        // Store origin position in Preferences
                        esp_task_wdt_reset();
                        storeOriginPosition(currentIndex, value);
                        esp_task_wdt_reset();
                    }
                    else
                    {
                        loadOriginPosition(currentIndex);
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
                        return;
                    }
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

                    // Store origin position in Preferences
                    esp_task_wdt_reset();
                    storeOriginPosition(currentIndex, value);
                    esp_task_wdt_reset();
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

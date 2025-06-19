#include "MAE3Encoder.h"
#include "MotorSpeedController.h"
#include "Pins.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <CLIManager.h>
#include <SPI.h>
#include <SimpleCLI.h>
#include <TMCStepper.h>
#include <esp_task_wdt.h>
#include <memory>

#define _MIN_SPEED       20
#define _FINE_MOVE_SPEED 12
#define _MAX_SPEED       200

struct MotorContext
{
    float   currentPosition;
    float   error;
    int32_t currentPositionPulses;
    int32_t targetPositionPulses;
    int32_t errorPulses;
};

std::unique_ptr<TMC5160Manager>       driver[NUM_DRIVERS]  = {nullptr};
std::unique_ptr<MotorSpeedController> motor[NUM_DRIVERS]   = {nullptr};
std::unique_ptr<MAE3Encoder>          encoder[NUM_DRIVERS] = {nullptr};

// Task handles
TaskHandle_t encoderUpdateTaskHandle = NULL;
TaskHandle_t motorUpdateTaskHandle   = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

static constexpr uint32_t SPI_CLOCK = 1000000;  // 1MHz SPI clock

static uint8_t currentIndex           = 0;  // Current driver index
static int32_t lastPulse[NUM_DRIVERS] = {0};

static float targetPosition[NUM_DRIVERS] = {0};

static bool driverEnabled[NUM_DRIVERS]             = {false};
static bool newTargetpositionReceived[NUM_DRIVERS] = {false};
static bool firstTime                              = true;

// Command history support
#define HISTORY_SIZE 10
static String commandHistory[HISTORY_SIZE];
static int    historyCount = 0;
static int    historyIndex = -1;  // -1 means not navigating

bool motorMoving[NUM_DRIVERS] = {false};

static float lastSpeed[NUM_DRIVERS] = {0.0f};

// Static variable to store total distance for speed profile calculations
static float initialTotalDistance[NUM_DRIVERS] = {0.0f};

void encoderUpdateTask(void* pvParameters);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);

bool         isValidNumber(const String& str);
void         setTargetPosition(String targetPos);
void         setMotorIndex(String motorIndex);
float        getMotorPosition();
MotorContext getMotorContext();
void         linearMotorUpdate();
int          calculateSteppedSpeed(float progressPercent, int minSpeed, int maxSpeed, float segmentSizePercent);
void         demonstrateSpeedProfile();
void         motorStopAndSavePosition(String callerFunctionName);
void         clearLine();
void         printSerial();
void         printMotorStatus();
void         resetMotorState(uint8_t motorIndex);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    // Initialize SPI
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);

    Serial.begin(115200);
    delay(1000);
    while (!Serial)
        delay(10);

    // Initialize CLI
    initializeCLI();

    // Initialize and print system diagnostics
    SystemDiagnostics::initialize();
    SystemDiagnostics::printSystemInfo();
    SystemDiagnostics::printSystemStatus();

    // Initialize TMC5160 drivers
    for (uint8_t index = 0; index < NUM_DRIVERS; index++)
    {
        // Create driver
        driver[index] = std::unique_ptr<TMC5160Manager>(new TMC5160Manager(index, DriverPins::CS[index]));
        driver[index]->begin();

        // Test connection for each driver
        if (driver[index]->testConnection(true))
        {
            driverEnabled[index] = true;
            // Configure motor parameters
            if (index == (uint8_t)MotorType::LINEAR)
                driver[index]->configureDriver_Nema11_1004H(true);
            else
                driver[index]->configureDriver_Pancake();

            // Create motor controller
            motor[index] = std::unique_ptr<MotorSpeedController>(new MotorSpeedController(
                index, *driver[index], DriverPins::DIR[index], DriverPins::STEP[index], DriverPins::EN[index]));

            // Initialize all motor controllers
            motor[index]->begin();

            // Create pwm encoder
            encoder[index] = std::unique_ptr<MAE3Encoder>(new MAE3Encoder(EncoderPins::SIGNAL[index], index));

            // Initialize all encoders
            encoder[index]->begin();
        }
    }

    if (!driverEnabled[0] && !driverEnabled[1] && !driverEnabled[2] && !driverEnabled[3])
    {
        Serial.printf("[Error][Setup] All drivers are disabled and the system is not operational. After powering on the system, "
                      "please reset it.\r\n");

        for (;;)
            delay(1000);
    }

    xTaskCreatePinnedToCore(encoderUpdateTask, "EncoderUpdateTask", 2048, NULL, 5, &encoderUpdateTaskHandle, 1);  // Core 1
    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 2048, NULL, 3, &motorUpdateTaskHandle, 1);        // Core 0
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 2048, NULL, 2, &serialReadTaskHandle, 0);           // Core 0
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 2048, NULL, 1, &serialPrintTaskHandle, 0);        // Core 0

    esp_task_wdt_add(encoderUpdateTaskHandle);  // Register with WDT
    esp_task_wdt_add(motorUpdateTaskHandle);    // Register with WDT
    esp_task_wdt_add(serialReadTaskHandle);     // Register with WDT
    esp_task_wdt_add(serialPrintTaskHandle);    // Register with WDT

    Serial.print(F("\r\n"));
    Serial.flush();
}

void loop()
{
    // Handle movement complete outside ISR
    if (motor[currentIndex])
    {
        motor[currentIndex]->handleMovementComplete();
    }

    esp_task_wdt_reset();
    vTaskDelay(1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool isValidNumber(const String& str)
{
    bool dotSeen = false;
    int  start   = (str[0] == '-') ? 1 : 0;

    if (start == 1 && str.length() == 1)
        return false;  // Not just "-"

    for (size_t i = start; i < str.length(); i++)
    {
        char c = str[i];
        if (c == '.')
        {
            if (dotSeen)
                return false;  // Only one dot is allowed
            dotSeen = true;
        }
        else if (!isDigit(c))
        {
            return false;
        }
    }

    return true;
}

// Target position is in um or degrees
void setTargetPosition(String targetPos)
{
    // 1. Validate the string
    if (!isValidNumber(targetPos))
    {
        Serial.print(F("[Error][SetTargetPosition] Target position must be a valid number.\r\n"));
        return;
    }

    // 2. Convert to decimal
    float value = targetPos.toFloat();

    // 3. Check based on currentIndex
    MotorType type = motor[currentIndex]->getMotorType();
    if (type == MotorType::LINEAR)
    {
        if (value < -2000.0f || value > 2000.0f)
        {
            Serial.print(
                F("[Error][SetTargetPosition] Target position must be between -2000.0 and 2000.0 (um) for linear motor.\r\n"));
            return;
        }
    }
    else if (type == MotorType::ROTATIONAL)
    {
        if (value < 0.1f || value > 359.9f)
        {
            Serial.print(
                F("[Error][SetTargetPosition] Target position must be between 0.1 and 359.9 (deg) for rotary motor.\r\n"));
            return;
        }
    }
    else
    {
        Serial.print(F("[Error][SetTargetPosition] Invalid motor number. Must be between 1 and 4.\r\n"));
        return;
    }

    // ✅ The value is valid
    if (currentIndex == (uint8_t)MotorType::LINEAR)
        driver[currentIndex]->configureDriver_Nema11_1004H(true);
    else
        driver[currentIndex]->configureDriver_Pancake();

    targetPosition[currentIndex]            = value;
    newTargetpositionReceived[currentIndex] = true;

    if (firstTime)  // Only print the first time (to avoid printing the error message before the motor is initialized)
        firstTime = false;

    Serial.printf("[Info][SetTargetPosition] Target position set to %.2f for motor %d.\r\n", value, currentIndex + 1);
}

// Motor number is 1-4 (0-3)
void setMotorIndex(String motorIndex)
{
    // 1. Check that it contains only numbers
    for (size_t i = 0; i < motorIndex.length(); i++)
    {
        if (!isDigit(motorIndex[i]))
        {
            Serial.print(F("[Error][SetMotorIndex] Motor index must be a number between 1 and 4.\r\n"));
            return;
        }
    }

    // 2. Convert to integer
    int index = motorIndex.toInt();
    if (index < 1 || index > 4)
    {
        Serial.print(F("[Error][setMotorIndex] Motor index must be between 1 and 4.\r\n"));
        return;
    }

    // 3. Check if the motor is enabled (assuming: we have an array or function to check the status)
    if (!driverEnabled[index - 1])
    {
        Serial.printf("[Warning][SetMotorIndex] Motor %d is currently disabled.\r\n", index);
        return;
    }

    currentIndex = index - 1;
    Serial.printf("[Info][SetMotorIndex] Motor %d selected.\r\n", index);
}

float getMotorPosition()
{
    MotorType      type   = motor[currentIndex]->getMotorType();
    EncoderContext encCtx = encoder[currentIndex]->getEncoderContext();
    return (type == MotorType::LINEAR) ? encCtx.total_travel_um : encCtx.position_degrees;
}

MotorContext getMotorContext()
{
    EncoderContext encCtx = encoder[currentIndex]->getEncoderContext();
    MotorContext   motCtx;
    MotorType      type = motor[currentIndex]->getMotorType();

    motCtx.currentPosition = (type == MotorType::LINEAR) ? encCtx.total_travel_um : encCtx.position_degrees;
    motCtx.error = motor[currentIndex]->calculateSignedPositionError(targetPosition[currentIndex], motCtx.currentPosition);

    if (type == MotorType::ROTATIONAL)
        motCtx.error = motor[currentIndex]->wrapAngle180(motCtx.error);

    motCtx.currentPositionPulses = encoder[currentIndex]->umToPulses(motCtx.currentPosition);
    motCtx.targetPositionPulses  = encoder[currentIndex]->umToPulses(targetPosition[currentIndex]);
    motCtx.errorPulses           = encoder[currentIndex]->umToPulses(motCtx.error);

    return motCtx;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void encoderUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(1);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    static bool isDriverConnectedMessageShown = false;

    while (true)
    {
        bool isEnabled = driverEnabled[currentIndex];

        if (!isEnabled && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.printf("[Error][EncoderUpdateTask] Motor %d connection failed!\r\n", currentIndex + 1);
        }
        else if (isEnabled && isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = false;
        }

        for (uint8_t ei = 0; ei < NUM_DRIVERS; ei++)
        {
            if (encoder[ei] == nullptr)
                continue;

            if (ei == currentIndex)
            {
                if (encoder[ei]->isDisabled())
                    encoder[ei]->enable();
            }
            else
            {
                if (encoder[ei]->isEnabled())
                    encoder[ei]->disable();
            }
        }

        if (isEnabled && encoder[currentIndex] != nullptr)
        {
            encoder[currentIndex]->processPWM();
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void motorUpdateTask(void* pvParameters)
{
    const uint8_t    MOTOR_UPDATE_TIME = 4;
    const TickType_t xFrequency        = pdMS_TO_TICKS(MOTOR_UPDATE_TIME);
    TickType_t       xLastWakeTime     = xTaskGetTickCount();

    static bool isDriverConnectedMessageShown = false;
    static int  lastReportedIndex             = -1;

    while (true)
    {
        bool isEnabled = driverEnabled[currentIndex];

        if (!isEnabled && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.printf("[Error][MotorUpdateTask] Motor %d connection failed!\r\n", currentIndex + 1);
        }
        else if (isEnabled)
        {
            isDriverConnectedMessageShown = false;

            if (motor[currentIndex] != nullptr)
            {
                if (lastReportedIndex != currentIndex)
                {
                    printMotorStatus();
                    lastReportedIndex = currentIndex;
                }

                linearMotorUpdate();
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void linearMotorUpdate()
{
    if (!newTargetpositionReceived[currentIndex])
        return;

    if (motor[currentIndex] == nullptr)
        return;

    static const float   POSITION_THRESHOLD_UM_FORWARD = 0.1f;              // Acceptable error range
    static const float   POSITION_THRESHOLD_UM_REVERSE = 0.3f;              // Acceptable error range
    static const float   FINE_MOVE_THRESHOLD_UM        = 2.5f;              // Fine movement threshold
    static const int32_t MAX_MICRO_MOVE_PULSE_FORWARD  = 5;                 // Maximum correction pulses
    static const int32_t MAX_MICRO_MOVE_PULSE_REVERSE  = 7;                 // Maximum correction pulses
    static const int     MIN_SPEED                     = _MIN_SPEED;        // Start and end speed
    static const int     MAX_SPEED                     = _MAX_SPEED;        // Maximum speed
    static const int     FINE_MOVE_SPEED               = _FINE_MOVE_SPEED;  // Fine movement speed
    static const float   SEGMENT_SIZE_PERCENT          = 5.0f;              // 5% segments for speed changes

    // Add debug logging for state variables
    // Serial.printf("[Debug][MotorUpdate] State check - motorMoving: %d, newTargetReceived: %d, initialDistance: %.2f\r\n",
    // motorMoving[currentIndex], newTargetpositionReceived[currentIndex], initialTotalDistance[currentIndex]);

    if (!motorMoving[currentIndex])  // Only when motor is stopped
    {
        MotorContext motCtx = getMotorContext();

        float absError = fabs(motCtx.error);
        float target   = targetPosition[currentIndex];

        int32_t targetPulse  = encoder[currentIndex]->umToPulses(target);
        int32_t currentPulse = encoder[currentIndex]->umToPulses(motCtx.currentPosition);
        int32_t deltaPulse   = targetPulse - currentPulse;

        float   positionThreshold = (motCtx.error > 0) ? POSITION_THRESHOLD_UM_FORWARD : POSITION_THRESHOLD_UM_REVERSE;
        int32_t maxMicroMovePulse = (motCtx.error > 0) ? MAX_MICRO_MOVE_PULSE_FORWARD : MAX_MICRO_MOVE_PULSE_REVERSE;

        // Check if we've reached the target position
        if (absError <= positionThreshold && abs(deltaPulse) < maxMicroMovePulse)
        {
            motorStopAndSavePosition("MotorUpdate");
            return;
        }

        // Set movement direction
        motor[currentIndex]->setDirection(motCtx.error > 0);

        if (!motor[currentIndex]->isMotorEnabled())
            motor[currentIndex]->motorEnable(true);

        // Register callback for movement completion
        motor[currentIndex]->attachOnComplete([]() { motorMoving[currentIndex] = false; });

        // Limit pulses for micro-movement
        if (abs(deltaPulse) > maxMicroMovePulse)
            deltaPulse = (deltaPulse > 0) ? maxMicroMovePulse : -maxMicroMovePulse;

        // Calculate speed based on smooth stepped profile
        int moveSpeed;
        if (absError <= FINE_MOVE_THRESHOLD_UM)
        {
            // Fine movement - low speed
            moveSpeed = FINE_MOVE_SPEED;
        }
        else
        {
            // Store total distance at the beginning of movement
            if (initialTotalDistance[currentIndex] == 0)
            {
                initialTotalDistance[currentIndex] = absError;
            }

            // Calculate progress percentage (0.0 to 1.0)
            float progressPercent = 1.0f - (absError / initialTotalDistance[currentIndex]);

            // Calculate speed based on stepped profile
            moveSpeed = calculateSteppedSpeed(progressPercent, MIN_SPEED, MAX_SPEED, SEGMENT_SIZE_PERCENT);
        }

        // Execute movement
        Serial.printf(
            "[Info][MotorUpdate] deltaPulse = %ld, error = %.2f um, speed = %d, progress = %.1f%%, isMotorEnabled = %d\n",
            (long)deltaPulse, motCtx.error, moveSpeed, (1.0f - (absError / initialTotalDistance[currentIndex])) * 100.0f,
            motor[currentIndex]->isMotorEnabled());

        motor[currentIndex]->move(deltaPulse, moveSpeed, lastSpeed[currentIndex]);
        lastSpeed[currentIndex]   = moveSpeed;
        motorMoving[currentIndex] = true;
    }
    else
    {
        // When motor is moving, don't reset the total distance
        // Only reset when movement is complete in motorStopAndSavePosition
    }
}

// Helper function to calculate stepped speed profile
int calculateSteppedSpeed(float progressPercent, int minSpeed, int maxSpeed, float segmentSizePercent)
{
    // Ensure progress is within bounds
    if (progressPercent <= 0.0f)
        return minSpeed;
    if (progressPercent >= 1.0f)
        return minSpeed;

    // Calculate which segment we're in (0 to 19 for 5% segments)
    int segment = static_cast<int>(progressPercent * 100.0f / segmentSizePercent);

    // Calculate total number of segments
    int totalSegments = static_cast<int>(100.0f / segmentSizePercent);

    // Calculate speed range
    int speedRange = maxSpeed - minSpeed;

    // Calculate speed based on segment position
    int speed;
    if (segment <= totalSegments / 2)
    {
        // Ramp up phase (first half)
        float rampUpProgress = static_cast<float>(segment) / (totalSegments / 2.0f);
        speed                = minSpeed + static_cast<int>(speedRange * rampUpProgress);
    }
    else
    {
        // Ramp down phase (second half)
        float rampDownProgress = static_cast<float>(segment - totalSegments / 2) / (totalSegments / 2.0f);
        speed                  = maxSpeed - static_cast<int>(speedRange * rampDownProgress);
    }

    // Ensure speed is within bounds
    if (speed < minSpeed)
        speed = minSpeed;
    if (speed > maxSpeed)
        speed = maxSpeed;

    return speed;
}

// Function to demonstrate speed profile (for testing)
void demonstrateSpeedProfile()
{
    Serial.println(F("\r\n=== Speed Profile Demonstration ==="));
    Serial.println(F("Progress% | Segment | Speed"));
    Serial.println(F("----------|---------|------"));

    const int   minSpeed    = _MIN_SPEED;
    const int   maxSpeed    = _MAX_SPEED;
    const float segmentSize = 5.0f;

    for (int i = 0; i <= 100; i += 5)
    {
        float progress = i / 100.0f;
        int   segment  = static_cast<int>(progress * 100.0f / segmentSize);
        int   speed    = calculateSteppedSpeed(progress, minSpeed, maxSpeed, segmentSize);

        Serial.printf("%8d%% | %7d | %5d\r\n", i, segment, speed);
    }
    Serial.println(F("=====================================\r\n"));
}

void motorStopAndSavePosition(String callerFunctionName)
{
    if (motor[currentIndex] == nullptr)
        return;

    callerFunctionName = "[" + callerFunctionName + "]";
    Serial.println(callerFunctionName);

    if (callerFunctionName == "MotorUpdate")
        Serial.println(F(" - [Info][MotorStopAndSavePosition] Reached target. Stopping..."));

    // Reset all state variables that prevent subsequent movements
    newTargetpositionReceived[currentIndex] = false;
    motorMoving[currentIndex]               = false;  // ✅ CRITICAL: Reset motor moving flag
    initialTotalDistance[currentIndex]      = 0.0f;   // ✅ Reset distance for speed profile
    lastSpeed[currentIndex]                 = 0.0f;

    motor[currentIndex]->stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (callerFunctionName == "MotorUpdate")
        Serial.println(F(" - [Info][MotorStopAndSavePosition] Final correction within threshold. Done."));

    printMotorStatus();
    printSerial();
}

// Emergency reset function to clear all motor state variables
void resetMotorState(uint8_t motorIndex)
{
    if (motorIndex >= NUM_DRIVERS)
        return;

    Serial.printf("[Info][ResetMotorState] Resetting motor %d state variables\r\n", motorIndex + 1);

    // Reset all state variables
    newTargetpositionReceived[motorIndex] = false;
    motorMoving[motorIndex]               = false;
    lastSpeed[motorIndex]                 = 0.0f;

    // Reset motor controller state
    if (motor[motorIndex] != nullptr)
    {
        motor[motorIndex]->stop();
        motor[motorIndex]->motorEnable(false);
    }

    Serial.printf("[Info][ResetMotorState] Motor %d state reset complete\r\n", motorIndex + 1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void clearLine()
{
    Serial.print(F("\r"));  // Go to the beginning of the line
    Serial.print(F("> "));
    for (int i = 0; i < 50; ++i)
        Serial.print(F(" "));  // More space
    Serial.print(F("\r> "));
}

void serialReadTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(100);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    String           inputBuffer   = "";
    String           lastInput     = "";

    for (;;)
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
                continue;
            }
            if (escState == 1 && c == '[')
            {
                escState = 2;
                esp_task_wdt_reset();
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
                        clearLine();
                        Serial.print(inputBuffer);
                    }
                    else if (historyIndex == 0)
                    {
                        historyIndex = -1;
                        inputBuffer  = lastInput;
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));
                        Serial.print(inputBuffer);
                    }
                    escState = 0;
                    esp_task_wdt_reset();
                    continue;
                }
                escState = 0;
                esp_task_wdt_reset();
                continue;
            }

            // Handle Enter
            if (c == '\n')
            {
                if (inputBuffer.length() > 0)
                {
                    Serial.print(F("\r\n"));
                    Serial.print(F("# "));
                    Serial.print(inputBuffer.c_str());
                    Serial.print(F("\r\n"));
                    cli.parse(inputBuffer);
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
            else if (c == 'i')
            {
                if (motor[currentIndex] != nullptr)
                {
                    motor[currentIndex]->setDirection(true);
                    motor[currentIndex]->move(10, 100, 0.0f);
                }
                else
                    Serial.print(F("[Error][SerialReadTask] Motor not connected\r\n"));

                esp_task_wdt_reset();
                continue;
            }
            else if (c == 'k')
            {
                if (motor[currentIndex] != nullptr)
                {
                    motor[currentIndex]->setDirection(false);
                    motor[currentIndex]->move(10, 100, 0.0f);
                }
                else
                    Serial.print(F("[Error][SerialReadTask] Motor not connected\r\n"));

                esp_task_wdt_reset();
                continue;
            }
            else
            {
                inputBuffer += c;  // Add character to buffer
                Serial.print(c);
                if (historyIndex == -1)
                    lastInput = inputBuffer;
            }
            esp_task_wdt_reset();
        }

        if (cli.available())
        {
            Command c = cli.getCmd();

            // Handle motor commands
            if (c == cmdMotor)
            {
                // Get motor number
                if (c.getArgument("n").isSet())
                {
                    String motorIndexStr = c.getArgument("n").getValue();
                    setMotorIndex(motorIndexStr);
                }
                else
                {
                    Serial.print(F("ERROR: Motor number (-n) requires a value\r\n"));
                    esp_task_wdt_reset();
                    continue;
                }

                // Handle Get motor position command
                if (c.getArgument("c").isSet())
                {
                    float position = getMotorPosition();
                    Serial.print(F("*"));
                    Serial.print(currentIndex);
                    Serial.print(F("#"));
                    Serial.print(position, 2);
                    Serial.print(F("#\r\n"));
                    esp_task_wdt_reset();
                    continue;
                }

                // Handle position commands
                if (c.getArgument("p").isSet())
                {
                    String targetPosition = c.getArgument("p").getValue();
                    setTargetPosition(targetPosition);
                    esp_task_wdt_reset();
                    continue;
                }
            }
            else if (c == cmdRestart)
            {
                motorStopAndSavePosition("CmdRestart");
                Serial.print(F("Restarting...\r\n"));
                vTaskDelay(pdMS_TO_TICKS(1000));
                ESP.restart();
            }
            else if (c == cmdStop)
            {
                motorStopAndSavePosition("CmdStop");
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdShow)
            {
                printSerial();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdDrive)
            {
                driver[currentIndex]->logDriverStatus();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdSpeedProfile)
            {
                demonstrateSpeedProfile();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdReset)
            {
                resetMotorState(currentIndex);
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.print(cli.toString());
                Serial.print(F("\r\n"));
            }
        }

        if (cli.errored())
        {
            String cmdError = cli.getError().toString();
            Serial.print(F("ERROR: "));
            Serial.print(cmdError);
            Serial.print(F("\r\n"));
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void serialPrintTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(300);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void printSerial()
{
    if (!driverEnabled[currentIndex] || encoder[currentIndex] == nullptr || (!Serial))
        return;

    EncoderContext encCtx = encoder[currentIndex]->getEncoderContext();
    MotorContext   motCtx = getMotorContext();

    MotorType type = motor[currentIndex]->getMotorType();
    String    unit = (type == MotorType::LINEAR ? "(um): " : "(deg): ");

    // if (fabs(encCtx.current_pulse - lastPulse[currentIndex]) > 1)
    //{
    //  Format all values into the buffer
    Serial.print(F("============================================================================================\r\n"));
    Serial.print(F("MOT: "));
    Serial.print((currentIndex));
    Serial.print(F("   "));
    Serial.print(F("DIR: "));
    Serial.print(encCtx.direction);
    Serial.print(F("   "));
    Serial.print(F("LAP: "));
    Serial.print(encCtx.lap_id);
    // Serial.print(F("\t"));
    // Serial.print(F("CUR PULSE: "));
    // Serial.print(encCtx.current_pulse);
    Serial.print(F("      "));
    Serial.print(F("CUR POS "));
    Serial.print(unit);
    Serial.print(motCtx.currentPosition);
    // Serial.print(F("\t"));
    // Serial.print(F("CUR POS (p): "));
    // Serial.print(motCtx.currentPositionPulses, 0);
    Serial.print(F("      "));
    Serial.print(F("TGT POS: "));
    Serial.print(unit);
    Serial.print(targetPosition[currentIndex]);
    // Serial.print(F("\t"));
    // Serial.print(F("TGT POS (p): "));
    // Serial.print(firstTime ? 0 : motCtx.targetPositionPulses, 0);
    Serial.print(F("      "));
    Serial.print(F("ERR: "));
    Serial.print(firstTime ? 0 : motCtx.error, 0);
    // Serial.print(F("\t"));
    // Serial.print(F("newTargetpositionReceived: "));
    // Serial.print(newTargetpositionReceived[currentIndex]);
    // Serial.print(F("\t"));
    // Serial.print(F("driverEnabled: "));
    // Serial.print(driverEnabled[currentIndex]);
    Serial.print(F("\r\n\r\n"));

    lastPulse[currentIndex] = encCtx.current_pulse;
    // }
}

void printMotorStatus()
{
    auto status = driver[currentIndex]->getDriverStatus();
    Serial.printf("\r\n[Info][PrintMotorStatus] Motor %d Status:\r\n", currentIndex + 1);
    Serial.printf(" - Current: %d mA\r\n", status.current);
    Serial.printf(" - Temperature: %d\r\n\r\n", status.temperature);
}

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

#define MAIN_ENCODER_TASK_DEBUG false

struct MotorContext
{
    float   currentPosition;
    float   error;
    int32_t currentPositionPulses;
    int32_t targetPositionPulses;
    int32_t errorPulses;
};

TMC5160Manager driver[4] = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]),
                            TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

MotorSpeedController motor[4] = {
    MotorSpeedController(0, driver[0], DriverPins::DIR[0], DriverPins::STEP[0], DriverPins::EN[0]),
    MotorSpeedController(1, driver[1], DriverPins::DIR[1], DriverPins::STEP[1], DriverPins::EN[1]),
    MotorSpeedController(2, driver[2], DriverPins::DIR[2], DriverPins::STEP[2], DriverPins::EN[2]),
    MotorSpeedController(3, driver[3], DriverPins::DIR[3], DriverPins::STEP[3], DriverPins::EN[3])};

MAE3Encoder encoder[4] = {MAE3Encoder(EncoderPins::SIGNAL[0], 0), MAE3Encoder(EncoderPins::SIGNAL[1], 1),
                          MAE3Encoder(EncoderPins::SIGNAL[2], 2), MAE3Encoder(EncoderPins::SIGNAL[3], 3)};

// Task handles
#if MAIN_ENCODER_TASK_DEBUG
TaskHandle_t encoderUpdateTaskHandle = NULL;
#endif
TaskHandle_t motorUpdateTaskHandle = NULL;
TaskHandle_t serialReadTaskHandle  = NULL;

static constexpr uint32_t SPI_CLOCK = 1000000;  // 1MHz SPI clock

static uint8_t currentIndex = 0;  // Current driver index
static int32_t lastPulse[4] = {0};

static float targetPosition[4] = {0};

static bool driverEnabled[4]             = {false};
static bool newTargetpositionReceived[4] = {false};
static bool firstTime                    = true;

// Command history support
#define HISTORY_SIZE 10
static String commandHistory[HISTORY_SIZE];
static int    historyCount = 0;
static int    historyIndex = -1;  // -1 means not navigating

bool motorMoving[4] = {false};

static float lastSpeed[4] = {0.0f};

// Static variable to store total distance for speed profile calculations
static float initialTotalDistance[4] = {0.0f};

// Static variable to store high water mark
static unsigned int highWaterMark = 0;

// ============================
// Function Declarations
// ============================
#if MAIN_ENCODER_TASK_DEBUG
void encoderUpdateTask(void* pvParameters);
#endif
void         motorUpdateTask(void* pvParameters);
void         serialReadTask(void* pvParameters);
bool         isValidNumber(const String& str);
void         setTargetPosition(String targetPos);
void         setMotorId(String motorId);
float        getMotorPosition();
MotorContext getMotorContext();
void         motorUpdateTask(void* pvParameters);
void         linearMotorUpdate();
void         rotaryMotorUpdate();
int          calculateSteppedSpeed(float progressPercent, int minSpeed, int maxSpeed, float segmentSizePercent);
void         demonstrateSpeedProfile();
void         motorStopAndSavePosition(String callerFunctionName);
void         resetMotorState(uint8_t id);
void         clearLine();
void         serialReadTask(void* pvParameters);
void         printSerial();
void         printMotorStatus();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    // Initialize SPI
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(115200);

    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);  // Add the current task (setup)
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    delay(1000);
    while (!Serial)
    {
        esp_task_wdt_reset();
        delay(10);
    }

#ifdef CONFIG_FREERTOS_CHECK_STACKOVERFLOW
    Serial.println(F("Stack overflow checking enabled"));
#else
    Serial.println(F("Stack overflow checking **NOT** enabled"));
#endif

    // Initialize CLI
    initializeCLI();

    // Initialize and print system diagnostics
    SystemDiagnostics::printSystemInfo();
    SystemDiagnostics::printSystemStatus();

    // for disable all drivers pins - for avoid conflict in SPI bus
    // Initialize CS pins and turn them off
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

            // Configure motor parameters
            if (index == (uint8_t)MotorType::LINEAR)
                driver[index].configureDriver_Nema11_1004H(true);
            else
                driver[index].configureDriver_Pancake();

            // Initialize all motor controllers
            motor[index].begin();

            // Initialize all encoders
            encoder[index].begin();
        }
    }

    // Check if any driver is enabled
    bool anyDriverEnabled = false;
    for (uint8_t index = 0; index < 4; index++)
        anyDriverEnabled = anyDriverEnabled || driverEnabled[index];

    if (!anyDriverEnabled)
    {
        Serial.println(F("[Error][Setup] All drivers are disabled. Please reset the system."));
        while (1)
        {
            esp_task_wdt_reset();
            delay(10);
        }
    }

    // Core 1 - For time-sensitive tasks (precise control)
#if MAIN_ENCODER_TASK_DEBUG
    xTaskCreatePinnedToCore(encoderUpdateTask, "EncoderUpdateTask", 4096, NULL, 5, &encoderUpdateTaskHandle, 1);
    esp_task_wdt_add(encoderUpdateTaskHandle);  // Register with WDT
#endif
    xTaskCreatePinnedToCore(motorUpdateTask, "MotorUpdateTask", 4096, NULL, 3, &motorUpdateTaskHandle, 1);
    esp_task_wdt_add(motorUpdateTaskHandle);  // Register with WDT

    // Core 0 - For less time-sensitive tasks (such as I/O)
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 4096, NULL, 2, &serialReadTaskHandle, 0);
    esp_task_wdt_add(serialReadTaskHandle);  // Register with WDT

    Serial.println();
    Serial.flush();
}

static bool diagnosticsRun = false;

void loop()
{
    // Handle movement complete outside ISR
    motor[currentIndex].handleMovementComplete();

    esp_task_wdt_reset();
    vTaskDelay(300);
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

// Target P. is in um or degrees (M100)
void setTargetPosition(String targetPos)
{
    // 1. Validate the string
    if (!isValidNumber(targetPos))
    {
        Serial.println(F("[Error][M100] Target P. invalid."));
        return;
    }

    // 2. Convert to decimal
    float value = targetPos.toFloat();

    // 3. Check based on currentIndex
    MotorType type = motor[currentIndex].getMotorType();
    if (type == MotorType::LINEAR)
    {
        if (value < -2000.0f || value > 2000.0f)
        {
            Serial.println(F("[Error][M100] Target P. limit (-/+)2000 (um) for linear motor."));
            return;
        }
    }
    else if (type == MotorType::ROTATIONAL)
    {
        if (value < 0.01f || value > 359.9f)
        {
            Serial.println(F("[Error][M100] Target P. limit 0.01/359.9 (deg) for rotary motor."));
            return;
        }
    }
    else
    {
        Serial.print(F("[Error][M100] Invalid Motor Id. limit: 1-"));
        Serial.println(4);
        return;
    }

    diagnosticsRun = false;  // set diagnosticsRun to false for new index

    // ✅ The value is valid
    if (currentIndex == (uint8_t)MotorType::LINEAR)
        driver[currentIndex].configureDriver_Nema11_1004H(true);
    else
        driver[currentIndex].configureDriver_Pancake();

    targetPosition[currentIndex]            = value;
    newTargetpositionReceived[currentIndex] = true;

    if (firstTime)  // Only print the first time (to avoid printing the error message before the motor is initialized)
        firstTime = false;

    Serial.print(F("[Info][M100] Target P. set to motor "));
    Serial.println(currentIndex + 1);
}

// Motor Id is 1-4 (0-3) (M101)
void setMotorId(String motorId)
{
    // 1. Check that it contains only numbers
    for (size_t i = 0; i < motorId.length(); i++)
    {
        if (!isDigit(motorId[i]))
        {
            Serial.print(F("[Error][M101] Invalid Motor Id. limit: 1-"));
            Serial.println(4);
            return;
        }
    }

    // 2. Convert to integer
    int index = motorId.toInt();
    if (index < 1 || index > 4)
    {
        Serial.print(F("[Error][M101] Invalid Motor Id. limit: 1-"));
        Serial.println(4);
        return;
    }

    currentIndex = index - 1;
    Serial.print(F("[Info][M101] Motor is selected: "));
    Serial.println(index);
}

// Get motor position (M102)
float getMotorPosition()
{
    MotorType      type   = motor[currentIndex].getMotorType();
    EncoderContext encCtx = encoder[currentIndex].getEncoderContext();
    return (type == MotorType::LINEAR) ? encCtx.total_travel_um : encCtx.position_degrees;
}

// Get motor context (M103)
MotorContext getMotorContext()
{
    MotorContext motCtx = {};  // Zero-initialize all members

    EncoderContext encCtx = encoder[currentIndex].getEncoderContext();
    MotorType      type   = motor[currentIndex].getMotorType();

    motCtx.currentPosition = (type == MotorType::LINEAR) ? encCtx.total_travel_um : encCtx.position_degrees;
    motCtx.error =
        motor[currentIndex].calculateSignedPositionError(targetPosition[currentIndex], motCtx.currentPosition);

    if (type == MotorType::ROTATIONAL)
        motCtx.error = motor[currentIndex].wrapAngle180(motCtx.error);

    //  Use appropriate pulse conversion based on motor type
    if (type == MotorType::LINEAR)
    {
        motCtx.currentPositionPulses = encoder[currentIndex].umToPulses(motCtx.currentPosition);
        motCtx.targetPositionPulses  = encoder[currentIndex].umToPulses(targetPosition[currentIndex]);
        motCtx.errorPulses           = encoder[currentIndex].umToPulses(motCtx.error);
    }
    else  // ROTATIONAL
    {
        motCtx.currentPositionPulses = encoder[currentIndex].degreesToPulses(motCtx.currentPosition);
        motCtx.targetPositionPulses  = encoder[currentIndex].degreesToPulses(targetPosition[currentIndex]);
        motCtx.errorPulses           = encoder[currentIndex].degreesToPulses(motCtx.error);
    }

    return motCtx;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if MAIN_ENCODER_TASK_DEBUG
// Motor Update Task (M104)
void encoderUpdateTask(void* pvParameters)  // amir
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(1000);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (diagnosticsRun)
        {
            encoder[currentIndex].processPWM(false);
            printSerial();
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
#endif
// Motor Update Task (M104)
void motorUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(2);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    static bool     isDriverConnectedMessageShown = false;
    static uint32_t counter                       = 0;

    while (1)
    {
#if MAIN_ENCODER_TASK_DEBUG
        // Run diagnostics first (only once)
        if (!diagnosticsRun)
        {
            encoder[currentIndex].diagnoseEncoderSignals();
            diagnosticsRun = true;
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }
#endif

        // if driver is not connected, show the message
        if (!driverEnabled[currentIndex] && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.print(F("[Error][M104] Driver's connection failed!: "));
            Serial.println(currentIndex + 1);
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // if driver is connected, don't show the message again
        isDriverConnectedMessageShown = driverEnabled[currentIndex] && isDriverConnectedMessageShown;

        // Enable/Disable encoder based on currentIndex
        for (uint8_t index = 0; index < 4; index++)
        {
            if (index != currentIndex && encoder[index].isEnabled())
                encoder[index].disable();
        }

        if (encoder[currentIndex].isDisabled())
            encoder[currentIndex].enable();

        encoder[currentIndex].processPWM();

        // Call appropriate motor update function based on motor type
        // Linear motor (index 0) and Rotary motors (indices 1-3)
        if (motor[currentIndex].getMotorType() == MotorType::LINEAR)
            linearMotorUpdate();

        else
            rotaryMotorUpdate();

        esp_task_wdt_reset();
        if (++counter % 1000 == 0)  // Only check and print once every 1000
            highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Linear Motor Update (M105)
void linearMotorUpdate()
{
    if (!newTargetpositionReceived[currentIndex])
        return;

    static const float   POSITION_THRESHOLD_UM_FORWARD = 0.1f;  // Acceptable error range
    static const float   POSITION_THRESHOLD_UM_REVERSE = 0.3f;  // Acceptable error range
    static const float   FINE_MOVE_THRESHOLD_UM        = 2.5f;  // Fine movement threshold
    static const int32_t MAX_MICRO_MOVE_PULSE_FORWARD  = 5;     // Maximum correction pulses
    static const int32_t MAX_MICRO_MOVE_PULSE_REVERSE  = 7;     // Maximum correction pulses
    static const int     MIN_SPEED                     = 20;    // Start and end speed
    static const int     MAX_SPEED                     = 200;   // Maximum speed
    static const int     FINE_MOVE_SPEED               = 12;    // Fine movement speed
    static const float   SEGMENT_SIZE_PERCENT          = 5.0f;  // 5% segments for speed changes

    if (!motorMoving[currentIndex])  // Only when motor is stopped
    {
        MotorContext motCtx = getMotorContext();

        float absError = fabs(motCtx.error);
        float target   = targetPosition[currentIndex];

        int32_t targetPulse  = encoder[currentIndex].umToPulses(target);
        int32_t currentPulse = encoder[currentIndex].umToPulses(motCtx.currentPosition);
        int32_t deltaPulse   = targetPulse - currentPulse;

        float   positionThreshold = (motCtx.error > 0) ? POSITION_THRESHOLD_UM_FORWARD : POSITION_THRESHOLD_UM_REVERSE;
        int32_t maxMicroMovePulse = (motCtx.error > 0) ? MAX_MICRO_MOVE_PULSE_FORWARD : MAX_MICRO_MOVE_PULSE_REVERSE;

        // Check if we've reached the Target P.
        if (absError <= positionThreshold && abs(deltaPulse) < maxMicroMovePulse)
        {
            motorStopAndSavePosition("M105");
            return;
        }

        // Set movement direction
        motor[currentIndex].setDirection(motCtx.error > 0);

        if (!motor[currentIndex].isEnabled())
            motor[currentIndex].enable();

        // Register callback for movement completion
        motor[currentIndex].attachOnComplete([]() { motorMoving[currentIndex] = false; });

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
        Serial.print(F("[Info][M105] Motor "));
        Serial.print(currentIndex + 1);
        Serial.print(F(", deltaPulse = "));
        Serial.print(deltaPulse);
        Serial.print(F(", error = "));
        Serial.print(motCtx.error, 2);
        Serial.print(F(" um, speed = "));
        Serial.print(moveSpeed);
        Serial.print(F(", progress = "));
        // Calculate progress with bounds checking
        float progressPercent = 0.0f;
        if (initialTotalDistance[currentIndex] > 0.0f)
        {
            progressPercent = 1.0f - (absError / initialTotalDistance[currentIndex]);
            progressPercent = constrain(progressPercent, 0.0f, 1.0f);  // Clamp between 0 and 1
        }
        Serial.println(progressPercent * 100.0f, 1);

        motor[currentIndex].move(deltaPulse, moveSpeed, lastSpeed[currentIndex]);
        lastSpeed[currentIndex]   = moveSpeed;
        motorMoving[currentIndex] = true;
    }
    else
    {
        // When motor is moving, don't reset the total distance
        // Only reset when movement is complete in motorStopAndSavePosition
    }
}

// Rotary Motor Update (M106)
void rotaryMotorUpdate()
{
    if (!newTargetpositionReceived[currentIndex])
        return;

    // Rotary motor specific constants
    static const float POSITION_THRESHOLD_DEG_FORWARD =
        0.1f;  // Acceptable error range for rotary motors (increased from 0.01f)
    static const float   FINE_MOVE_THRESHOLD_DEG      = 1.0f;  // Fine movement threshold for rotary motors
    static const int32_t MAX_MICRO_MOVE_PULSE_FORWARD = 3;     // Maximum correction pulses for rotary
    static const int     MIN_SPEED                    = 8;     // Start and end speed
    static const int     MAX_SPEED                    = 8;     // Maximum speed
    static const int     FINE_MOVE_SPEED              = 8;     // Fine movement speed
    static const float   SEGMENT_SIZE_PERCENT         = 5.0f;  // 5% segments for speed changes

    // Validate Target P. is within rotary motor limits (0.01 to 359.9 degrees)
    float target = targetPosition[currentIndex];
    if (target < 0.01f || target > 359.9f)
    {
        Serial.print(F("[Error][M106] Target P. is out of range 0.01/359.9 (deg): "));
        Serial.println(target);
        newTargetpositionReceived[currentIndex] = false;
        return;
    }

    if (!motorMoving[currentIndex])  // Only when motor is stopped
    {
        MotorContext motCtx = getMotorContext();

        float absError = fabs(motCtx.error);

        // Convert degrees to pulses for rotary motor
        int32_t targetPulse  = encoder[currentIndex].degreesToPulses(target);
        int32_t currentPulse = encoder[currentIndex].degreesToPulses(motCtx.currentPosition);
        int32_t deltaPulse   = targetPulse - currentPulse;

        // Use symmetric thresholds for rotary motors
        float   positionThreshold = POSITION_THRESHOLD_DEG_FORWARD;
        int32_t maxMicroMovePulse = MAX_MICRO_MOVE_PULSE_FORWARD;

        // Check if we've reached the Target P.
        if (absError <= positionThreshold && abs(deltaPulse) < maxMicroMovePulse)
        {
            Serial.print(F("[DEBUG] Stop condition 1 met: absError="));
            Serial.print(absError, 3);
            Serial.print(F(", threshold="));
            Serial.print(positionThreshold, 3);
            Serial.print(F(", deltaPulse="));
            Serial.print(abs(deltaPulse));
            Serial.print(F(", maxPulse="));
            Serial.println(maxMicroMovePulse);
            motorStopAndSavePosition("M106");
            return;
        }

        // Additional stop condition: if error is very small and no movement is needed
        if (absError <= 0.05f && abs(deltaPulse) <= 1)
        {
            Serial.print(F("[DEBUG] Stop condition 2 met: absError="));
            Serial.print(absError, 3);
            Serial.print(F(", deltaPulse="));
            Serial.println(abs(deltaPulse));
            motorStopAndSavePosition("M106");
            return;
        }

        // Set movement direction
        motor[currentIndex].setDirection(motCtx.error > 0);

        if (!motor[currentIndex].isEnabled())
            motor[currentIndex].enable();

        // Register callback for movement completion
        motor[currentIndex].attachOnComplete([]() { motorMoving[currentIndex] = false; });

        // Limit pulses for micro-movement
        if (abs(deltaPulse) > maxMicroMovePulse)
            deltaPulse = (deltaPulse > 0) ? maxMicroMovePulse : -maxMicroMovePulse;

        // Calculate speed based on smooth stepped profile
        int moveSpeed;
        if (absError <= FINE_MOVE_THRESHOLD_DEG)
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
        Serial.print(F("[Info][M106] Motor "));
        Serial.print(currentIndex + 1);
        Serial.print(F(", deltaPulse = "));
        Serial.print(deltaPulse);
        Serial.print(F(", error = "));
        Serial.print(motCtx.error, 2);
        Serial.print(F(" deg, speed = "));
        Serial.print(moveSpeed);
        Serial.print(F(", progress = "));
        // Calculate progress with bounds checking
        float progressPercent = 0.0f;
        if (initialTotalDistance[currentIndex] > 0.0f)
        {
            progressPercent = 1.0f - (absError / initialTotalDistance[currentIndex]);
            progressPercent = constrain(progressPercent, 0.0f, 1.0f);  // Clamp between 0 and 1
        }
        Serial.println(progressPercent * 100.0f, 1);

        motor[currentIndex].move(deltaPulse, moveSpeed, lastSpeed[currentIndex]);
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

    const int   minSpeed    = 20;
    const int   maxSpeed    = 200;
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

// Motor Stop and Save Position (M107)
void motorStopAndSavePosition(String callerFunctionName)
{
    callerFunctionName = "[" + callerFunctionName + "]";
    Serial.println(callerFunctionName);

    if (callerFunctionName == "M105" || callerFunctionName == "M106")
        Serial.println(F(" - [Info][M107] Reached target. Stopping..."));

    // Reset all state variables that prevent subsequent movements
    newTargetpositionReceived[currentIndex] = false;
    motorMoving[currentIndex]               = false;  // ✅ CRITICAL: Reset motor moving flag
    initialTotalDistance[currentIndex]      = 0.0f;   // ✅ Reset distance for speed profile
    lastSpeed[currentIndex]                 = 0.0f;

    motor[currentIndex].stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (callerFunctionName == "M105" || callerFunctionName == "M106")
        Serial.println(F(" - [Info][M107] Final correction within threshold. Done."));

    printMotorStatus();
    printSerial();
}

// Emergency reset function to clear all motor state variables (M108)
void resetMotorState(uint8_t id)
{
    if (id >= 4)
        return;

    Serial.print(F("[Info][M108] Resetting motor state variables: "));
    Serial.println(id + 1);

    // Reset all state variables
    newTargetpositionReceived[id] = false;
    motorMoving[id]               = false;
    lastSpeed[id]                 = 0.0f;

    // Reset motor controller state
    motor[id].stop();
    motor[id].disable();

    Serial.print(F("[Info][M108] Motor state reset complete: "));
    Serial.println(id + 1);
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

// Serial Read Task (M109)
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
                motor[currentIndex].setDirection(true);
                motor[currentIndex].move(10, 100, 0.0f);

                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            else if (c == 'k')
            {
                motor[currentIndex].setDirection(false);
                motor[currentIndex].move(10, 100, 0.0f);

                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                else
                {
                    Serial.println(F("ERROR: Motor Id (-n) requires a value"));
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }

                // Handle position commands
                if (c.getArgument("p").isSet())
                {
                    String targetPosition = c.getArgument("p").getValue();
                    setTargetPosition(targetPosition);
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
            }
            else if (c == cmdRestart)
            {
                motorStopAndSavePosition("CmdRestart");
                Serial.println(F("Restarting..."));
                vTaskDelay(pdMS_TO_TICKS(1000));
                ESP.restart();
            }
            else if (c == cmdStop)
            {
                motorStopAndSavePosition("CmdStop");
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            else if (c == cmdShow)
            {
                printSerial();
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            else if (c == cmdDrive)
            {
                driver[currentIndex].logDriverStatus();
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            else if (c == cmdSpeedProfile)
            {
                demonstrateSpeedProfile();
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            else if (c == cmdReset)
            {
                resetMotorState(currentIndex);
                esp_task_wdt_reset();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
// Print Serial (M110)
void printSerial()
{
    Serial.print(F("[M110] HighWaterMark: "));
    Serial.print(highWaterMark);
    Serial.print(F(" words ("));
    Serial.print(highWaterMark * 4);
    Serial.print(F(" bytes)\n"));

    EncoderContext encCtx = encoder[currentIndex].getEncoderContext();
    MotorContext   motCtx = getMotorContext();

    MotorType type = motor[currentIndex].getMotorType();
    String    unit = (type == MotorType::LINEAR ? "(um): " : "(deg): ");

    // if (fabs(encCtx.current_pulse - lastPulse[currentIndex]) > 1)
    //{
    //  Format all values into the buffer
    Serial.print(F("============================================================================================\r\n"));
    Serial.print(F("MOT: "));
    Serial.print((currentIndex + 1));
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

// Print Motor Status (M111)
void printMotorStatus()
{
    auto status = driver[currentIndex].getDriverStatus();
    Serial.print(F("\r\n[Info][M111] Motor "));
    Serial.print(currentIndex + 1);
    Serial.print(F(" Status:\r\n"));
    Serial.print(F(" - Current: "));
    Serial.print(status.current);
    Serial.print(F(" mA\r\n"));
    Serial.print(F(" - Temperature: "));
    Serial.print(status.temperature);
    Serial.println(F("\r\n"));
}

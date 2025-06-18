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
static bool isVeryShortDistance                    = false;
static bool firstTime                              = true;

// Command history support
#define HISTORY_SIZE 10
static String commandHistory[HISTORY_SIZE];
static int    historyCount = 0;
static int    historyIndex = -1;  // -1 means not navigating

bool motorMoving[NUM_DRIVERS] = {false};

static float lastSpeed[NUM_DRIVERS] = {0.0f};

void         setTargetPosition(String targetPosition);  // Target position is in um or degrees
void         setMotorIndex(String motorIndex);          // Motor number is 1-4 (0-3)
float        getMotorPosition();                        // Get current motor position
MotorContext getMotorContext();                         // Get motor context

void encoderUpdateTask(void* pvParameters);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);

void printSerial();
void MotorUpdate();
void motorStopAndSavePosition();
void printMotorStatus();

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
        if (driver[index]->testConnection())
        {
            driverEnabled[index] = true;
            Serial.printf("[Setup] Driver %d connected successfully\r\n", index);

            // Get and print driver status
            auto status = driver[index]->getDriverStatus();
            Serial.printf("[Setup] Driver %d Status:\r\n", index);
            Serial.printf(" - Version: 0x%08X\r\n", status.version);
            Serial.printf(" - Current: %d mA\r\n", status.current);
            Serial.printf(" - Temperature: %d\r\n\r\n", status.temperature);

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
        else
        {
            Serial.printf("[Warning][Setup] Driver %d connection failed!\r\n", index);
        }

        if (!driverEnabled[0] && !driverEnabled[1] && !driverEnabled[2] && !driverEnabled[3])
        {
            Serial.printf(
                "[Error][Setup] All drivers are disabled and the system is not operational. After powering on the system, "
                "please reset it.\r\n");

            for (;;)
                delay(1000);
        }
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

    Serial.printf("[OK][SetTargetPosition] Target position set to %.2f for motor %d.\r\n", value, currentIndex + 1);
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
    Serial.printf("[OK][SetMotorIndex] Motor %d selected.\r\n", index);
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

                MotorUpdate();
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MotorUpdate()
{
    if (!newTargetpositionReceived[currentIndex])
        return;

    if (motor[currentIndex] == nullptr)
        return;

    static const float   POSITION_THRESHOLD_UM_FORWARD = 0.1f;  // محدوده خطای قابل قبول
    static const float   POSITION_THRESHOLD_UM_REVERSE = 0.3f;  // محدوده خطای قابل قبول
    static const float   FINE_MOVE_THRESHOLD_UM        = 2.5f;  // شروع حرکت آهسته
    static const int32_t MAX_MICRO_MOVE_PULSE_FORWARD  = 5;     // حداکثر پالس اصلاحی
    static const int32_t MAX_MICRO_MOVE_PULSE_REVERSE  = 7;     // حداکثر پالس اصلاحی
    static const int     NORMAL_MOVE_SPEED             = 100;   // سرعت حرکت عادی
    static const int     FINE_MOVE_SPEED               = 15;    // سرعت حرکت دقیق

    if (!motorMoving[currentIndex])  // فقط وقتی موتور متوقف است
    {
        MotorContext motCtx = getMotorContext();

        float absError = fabs(motCtx.error);
        float target   = targetPosition[currentIndex];

        int32_t targetPulse  = encoder[currentIndex]->umToPulses(target);
        int32_t currentPulse = encoder[currentIndex]->umToPulses(motCtx.currentPosition);
        int32_t deltaPulse   = targetPulse - currentPulse;

        float   positionThreshold = (motCtx.error > 0) ? POSITION_THRESHOLD_UM_FORWARD : POSITION_THRESHOLD_UM_REVERSE;
        int32_t maxMicroMovePulse = (motCtx.error > 0) ? MAX_MICRO_MOVE_PULSE_FORWARD : MAX_MICRO_MOVE_PULSE_REVERSE;

        // شرایط رسیدن به موقعیت نهایی
        if (absError <= positionThreshold && abs(deltaPulse) < maxMicroMovePulse)
        {
            newTargetpositionReceived[currentIndex] = false;
            Serial.println("[Motor] Final correction within threshold. Done.");
            motorStopAndSavePosition();
            return;
        }

        // تنظیم جهت حرکت
        motor[currentIndex]->setDirection(motCtx.error > 0);

        if (!motor[currentIndex]->isMotorEnabled())
            motor[currentIndex]->motorEnable(true);

        // ثبت callback پس از اتمام حرکت
        motor[currentIndex]->attachOnComplete([]() { motorMoving[currentIndex] = false; });

        // محدود کردن پالس در micro-move
        if (abs(deltaPulse) > maxMicroMovePulse)
            deltaPulse = (deltaPulse > 0) ? maxMicroMovePulse : -maxMicroMovePulse;

        // انتخاب سرعت بر اساس فاصله تا هدف
        int moveSpeed = (absError <= FINE_MOVE_THRESHOLD_UM) ? FINE_MOVE_SPEED : NORMAL_MOVE_SPEED;

        // اجرای حرکت
        Serial.printf("[Motor] deltaPulse = %ld, error = %.2f um, speed = %d, isMotorEnabled = %d\n", (long)deltaPulse,
                      motCtx.error, moveSpeed, motor[currentIndex]->isMotorEnabled());

        motor[currentIndex]->move(deltaPulse, moveSpeed, lastSpeed[currentIndex]);
        lastSpeed[currentIndex]   = moveSpeed;
        motorMoving[currentIndex] = true;
    }
}

void motorStopAndSavePosition()
{
    if (motor[currentIndex] == nullptr)
        return;

    motor[currentIndex]->stop();

    lastSpeed[currentIndex] = 0.0f;

    newTargetpositionReceived[currentIndex] = false;

    Serial.print(F("[Motor] Reached target. Stopping...\r\n"));

    vTaskDelay(pdMS_TO_TICKS(1000));

    printSerial();
    printMotorStatus();
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
                newTargetpositionReceived[currentIndex] = false;
                if (motor[currentIndex] != nullptr)
                    motor[currentIndex]->stop();

                Serial.print(F("Restarting...\r\n"));
                vTaskDelay(pdMS_TO_TICKS(1000));
                ESP.restart();
            }
            else if (c == cmdStop)
            {
                newTargetpositionReceived[currentIndex] = false;
                motor[currentIndex]->stop();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdShow)
            {
                printSerial();
                driver[currentIndex]->logDriverStatus();
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
    return;
    auto status = driver[currentIndex]->getDriverStatus();
    Serial.printf("\r\n[MotorUpdateTask] Motor %d Status:\r\n", currentIndex);
    Serial.printf(" - Current: %d mA\r\n", status.current);
    Serial.printf(" - Temperature: %d\r\n\r\n", status.temperature);
}

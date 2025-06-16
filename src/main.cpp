#include "MAE3Encoder.h"
#include "MotorSpeedController.h"
#include "Pins.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <CLIManager.h>
#include <Helper.h>
#include <SPI.h>
#include <SimpleCLI.h>
#include <TMCStepper.h>
#include <esp_task_wdt.h>
#include <memory>

// Create drivers for all motors
std::unique_ptr<TMC5160Manager> driver[NUM_DRIVERS] = {nullptr};

// Create controllers for all motors
std::unique_ptr<MotorSpeedController> motor[NUM_DRIVERS] = {nullptr};

// Create pwm encoders for all motors
std::unique_ptr<MAE3Encoder> encoder[NUM_DRIVERS] = {nullptr};

// Task handles
TaskHandle_t encoderUpdateTaskHandle = NULL;
TaskHandle_t motorUpdateTaskHandle   = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;
TaskHandle_t serialPrintTaskHandle   = NULL;

static constexpr uint32_t SPI_CLOCK                   = 1000000;  // 1MHz SPI clock
static uint8_t            cmi                         = 0;        // Current driver index
static float              targetPosition[NUM_DRIVERS] = {0};
static bool               driverEnabled[NUM_DRIVERS]  = {false};

struct MotorContext
{
    float current_pos;
    float error;
    float current_pos_pulses;
    float target_position_pulses;
    float error_pulses;
};

MotorContext mc[NUM_DRIVERS];

bool command_received[NUM_DRIVERS] = {false};
bool is_set_motor_number           = false;
bool isVeryShortDistance           = false;

void    setTargetPosition(String targetPosition);  // Target position is in um or degrees
void    setMotorIndex(String motorIndex);          // Motor number is 1-4 (0-3)
int32_t getMotorPosition();

void encoderUpdateTask(void* pvParameters);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);

void printSerial();
void MotorUpdate();
void motorStopAndSavePosition();
void printMotorStatus();
void updateMotorContext();

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
    for (uint8_t di = 0; di < NUM_DRIVERS; di++)
    {
        // Create driver
        driver[di] = std::unique_ptr<TMC5160Manager>(new TMC5160Manager(di, DriverPins::CS[di]));
        driver[di]->begin();

        // Test connection for each driver
        if (driver[di]->testConnection())
        {
            driverEnabled[di] = true;
            Serial.printf("[Setup] Driver %d connected successfully\n", di);

            // Get and print driver status
            auto status = driver[di]->getDriverStatus();
            Serial.printf("[Setup] Driver %d Status:\n", di);
            Serial.printf(" - Version: 0x%08X\n", status.version);
            Serial.printf(" - Current: %d mA\n", status.current);
            Serial.printf(" - Temperature: %d\n\n", status.temperature);

            // Configure motor parameters
            if (di == (uint8_t)MotorType::LINEAR)
                driver[di]->configureDriver_Nema11_1004H();
            else
                driver[di]->configureDriver_Pancake();

            // Create motor controller
            motor[di] = std::unique_ptr<MotorSpeedController>(
                new MotorSpeedController(di, *driver[di], DriverPins::DIR[di], DriverPins::STEP[di], DriverPins::EN[di]));

            // Initialize all motor controllers
            motor[di]->begin();

            // Create pwm encoder
            encoder[di] = std::unique_ptr<MAE3Encoder>(new MAE3Encoder(EncoderPins::SIGNAL[di], di));

            // Initialize all encoders
            encoder[di]->begin();
        }
        else
        {
            String gFailed = Helper::redText("failed");
            Serial.printf("[Setup] Driver %d connection %s!\n", di, gFailed.c_str());
        }

        if (!driverEnabled[0] && !driverEnabled[1] && !driverEnabled[2] && !driverEnabled[3])
        {
            Serial.printf("[Setup] All drivers are disabled and the system is not operational. After powering on the system, "
                          "please reset it.\n");

            while (1)
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
}

void loop()
{
    // Print motor status every 2 seconds
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime >= 2000)
    {
        lastStatusTime = millis();
    }

    esp_task_wdt_reset();
    vTaskDelay(1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Target position is in um or degrees
void setTargetPosition(String target_Position)
{
    try
    {
        targetPosition[cmi]   = target_Position.toFloat();
        command_received[cmi] = true;
    }
    catch (const std::exception& e)
    {
        Serial.printf("[SetTargetPosition] Invalid target position! %s\n", e.what());
        return;
    }
}

// Motor number is 1-4 (0-3)
void setMotorIndex(String motorIndex)
{
    try
    {
        uint8_t motorIndexInt = motorIndex.toInt();
        if (motorIndexInt > NUM_DRIVERS || motorIndexInt == 0)
        {
            Serial.printf("[SetMotorIndex] Invalid motor number! %d\n", motorIndexInt);
            return;
        }
        cmi = motorIndexInt - 1;
    }
    catch (const std::exception& e)
    {
        Serial.printf("[SetMotorIndex] Invalid motor number! %s\n", e.what());
        return;
    }
}

int32_t getMotorPosition()
{
    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void encoderUpdateTask(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(1);
    TickType_t       xLastWakeTime = xTaskGetTickCount();

    // Show driver connection message only once
    static bool isDriverConnectedMessageShown = false;

    while (1)
    {
        if (!driver[cmi]->testConnection() && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.printf("[EncoderUpdateTask] Driver %d connection failed!\n", cmi);
        }

        if (driver[cmi]->testConnection())
            isDriverConnectedMessageShown = false;  // Reset the message shown flag

        for (uint8_t ei = 0; ei < NUM_DRIVERS; ei++)
        {
            if (encoder[ei] != nullptr)
            {
                if (ei != cmi)
                {
                    if (encoder[ei]->isEnabled())
                        encoder[ei]->disable();  // disable other encoder
                }
                else
                {
                    if (encoder[ei]->isDisabled())
                        encoder[ei]->enable();  // enable current encoder
                }
            }
        }

        if (driver[cmi]->testConnection() && encoder[cmi] != nullptr)
            encoder[cmi]->processPWM();

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
    // Show driver status only once
    static bool hasReportedDriverStatus = false;

    while (1)
    {
        // Show driver connection message only once

        if (!driver[cmi]->testConnection() && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.printf("[MotorUpdateTask] Driver %d connection failed!\n", cmi);
        }

        if (driver[cmi]->testConnection() && motor[cmi] != nullptr)
        {
            // Reset the message shown flag
            isDriverConnectedMessageShown = false;

            if (!hasReportedDriverStatus)
            {
                hasReportedDriverStatus = true;

                printMotorStatus();
            }

            MotorUpdate();
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command history support
#define HISTORY_SIZE 10
String commandHistory[HISTORY_SIZE];
int    historyCount = 0;
int    historyIndex = -1;  // -1 means not navigating
void   serialReadTask(void* pvParameters)
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
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));  // Overwrite any old chars
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
                        Serial.print(F("\r> "));
                        Serial.print(inputBuffer);
                        Serial.print(F("           \r> "));
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
                        else
                            historyCount = HISTORY_SIZE;
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
                motor[cmi]->moveForward();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == 'k')
            {
                motor[cmi]->moveBackward();
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
                    Serial.println(F("ERROR: Motor number (-n) requires a value"));
                    esp_task_wdt_reset();
                    continue;
                }

                // Handle Get motor position command
                if (c.getArgument("c").isSet())
                {
                    int32_t position = getMotorPosition();
                    Serial.print(F("*"));
                    Serial.print(cmi);
                    Serial.print(F("#"));
                    Serial.print(position);
                    Serial.println(F("#"));
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
                command_received[cmi] = false;
                motor[cmi]->stopMotor();
                delay(1000);
                Serial.println(F("Restarting..."));
                delay(1000);
                ESP.restart();
            }
            else if (c == cmdStop)
            {
                command_received[cmi] = false;
                motor[cmi]->stopMotor();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.println(cli.toString());
            }
        }

        if (cli.errored())
        {
            String cmdError = cli.getError().toString();
            Serial.print(F("ERROR: "));
            Serial.println(cmdError);
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

    while (1)
    {
        printSerial();
        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t last_pulse[NUM_DRIVERS] = {0};
void    printSerial()
{
    if (!driver[cmi]->testConnection() || encoder[cmi] == nullptr)
        return;

    EncoderContext ec = encoder[cmi]->getEncoderContext();
    updateMotorContext();

    if (fabs(ec.current_pulse - last_pulse[cmi]) > 1)
    {
        // Format all values into the buffer
        Serial.print(F("MOT: "));
        Serial.print((cmi));
        Serial.print(F("    "));
        Serial.print(F("DIR: "));
        Serial.print(ec.direction);
        Serial.print(F("    "));
        Serial.print(F("C PULSE: "));
        Serial.print(ec.current_pulse);
        Serial.print(F("    "));
        Serial.print(F("LAP: "));
        Serial.print(ec.lap_id);
        Serial.print(F("    "));
        Serial.print(F("C POS: "));
        Serial.print(mc[cmi].current_pos);
        Serial.print(F("    "));
        Serial.print(F("T POS (p): "));
        Serial.print(mc[cmi].target_position_pulses, 0);
        Serial.print(F("    "));
        Serial.print(F("C POS (p): "));
        Serial.print(mc[cmi].current_pos_pulses, 0);
        Serial.print(F("    "));
        Serial.print(F("ERR (p): "));
        Serial.print(mc[cmi].error_pulses, 0);
        Serial.println("\n\n");

        last_pulse[cmi] = ec.current_pulse;
    }
}

void updateMotorContext()
{
    EncoderContext ec = encoder[cmi]->getEncoderContext();

    mc[cmi].current_pos = (cmi == (uint8_t)MotorType::LINEAR) ? ec.total_travel_um : ec.position_degrees;
    mc[cmi].error       = motor[cmi]->calculateSignedPositionError(targetPosition[cmi], mc[cmi].current_pos);

    if (cmi != (uint8_t)MotorType::LINEAR)
        mc[cmi].error = motor[cmi]->wrapAngle180(mc[cmi].error);

    mc[cmi].current_pos_pulses     = encoder[cmi]->umToPulses(mc[cmi].current_pos);
    mc[cmi].target_position_pulses = encoder[cmi]->umToPulses(targetPosition[cmi]);
    mc[cmi].error_pulses           = encoder[cmi]->umToPulses(mc[cmi].error);
}

void MotorUpdate()
{
    if (!command_received[cmi])
        return;

    updateMotorContext();

    // Motor control based on error threshold
    if (isVeryShortDistance || abs(mc[cmi].error) > 0.1f)
    {
        isVeryShortDistance = false;

        // Set direction based on error sign
        motor[cmi]->setMotorDirection(mc[cmi].error > 0);

        if (!motor[cmi]->isMotorEnabled())
            motor[cmi]->motorEnable(true);

        // Update frequency based on error and target position
        motor[cmi]->updateMotorFrequency(mc[cmi].error_pulses, mc[cmi].target_position_pulses, mc[cmi].current_pos_pulses);
    }
    else
    {
        // Target reached - stop motor
        motorStopAndSavePosition();
    }
}

void motorStopAndSavePosition()
{
    motor[cmi]->stopMotor();
    command_received[cmi] = false;  // Reset command flag when target is reached or no command
    Serial.println(F("Motor Stop"));
    delay(1000);
    printMotorStatus();
}

void printMotorStatus()
{
    auto status = driver[cmi]->getDriverStatus();
    Serial.printf("\n[MotorUpdateTask] Motor %d Status:\n", cmi);
    Serial.printf("  -- Current: %d mA\n", status.current);
    Serial.printf("  -- Temperature: %d\n", status.temperature);

    // Get current speed based on motor index
    int16_t speed = motor[cmi]->getCurrentSpeed();
    Serial.printf("  -- Speed: %d\n\n", speed);
}
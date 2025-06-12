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
static uint8_t            currentIndex                = 0;
static float              targetPosition[NUM_DRIVERS] = {0};

bool command_received[NUM_DRIVERS] = {false};
bool is_set_motor_number           = false;
bool isVeryShortDistance           = false;

void    setTargetPosition(String targetPosition);  // Target position is in um or degrees
void    setMotorIndex(String motorNumber);         // Motor number is 1-4 (0-3)
int32_t getMotorPosition();

void encoderUpdateTask(void* pvParameters);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);
void serialPrintTask(void* pvParameters);

void printSerial();

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

    // Initialize and print system diagnostics
    SystemDiagnostics::initialize();
    SystemDiagnostics::printSystemInfo();
    SystemDiagnostics::printSystemStatus();

    // Initialize TMC5160 drivers
    for (uint8_t di = 0; di < NUM_DRIVERS; di++)
    {
        // Create driver
        driver[di] = std::unique_ptr<TMC5160Manager>(new TMC5160Manager(di, DriverPins::CS[di]));

        if (!driver[di]->begin())
        {
            Serial.printf("Failed to initialize TMC5160 driver %d\n", di + 1);
            while (1)
                delay(1000);
        }
    }

    // Test connection for each driver
    for (uint8_t di = 0; di < NUM_DRIVERS; di++)
    {
        if (driver[di]->testConnection())
        {
            Serial.printf("Driver %d connected successfully\n", di + 1);

            // Get and print driver status
            auto status = driver[di]->getDriverStatus();
            Serial.printf("Driver %d Status:\n", di + 1);
            Serial.printf("  Version: 0x%08X\n", status.version);
            Serial.printf("  Current: %d mA\n", status.current);
            Serial.printf("  Temperature: %d\n", status.temperature);

            // Configure motor parameters
            if (di == (uint8_t)MotorType::LINEAR)
                driver[di]->configureDriver_Nema11_1004H();
            else
                driver[di]->configureDriver_Pancake();
        }
        else
        {
            Serial.printf("Driver %d connection failed!\n", di + 1);
        }
    }

    // Initialize motors
    for (uint8_t mi = 0; mi < NUM_DRIVERS; mi++)
    {
        if (driver[mi]->testConnection())
        {
            // Create motor controller
            motor[mi] = std::unique_ptr<MotorSpeedController>(
                new MotorSpeedController(mi, *driver[mi], DriverPins::DIR[mi], DriverPins::STEP[mi], DriverPins::EN[mi]));

            // Initialize all motor controllers
            motor[mi]->begin();

            // Create pwm encoder
            encoder[mi] = std::unique_ptr<MAE3Encoder>(new MAE3Encoder(EncoderPins::SIGNAL[mi], mi));

            // Initialize all encoders
            encoder[mi]->begin();
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
void setTargetPosition(String targetPosition)
{
    try
    {
        targetPosition[currentIndex] = targetPosition.toFloat();
    }
    catch (const std::exception& e)
    {
        Serial.printf("ERROR: Invalid target position! %s\n", e.what());
        return;
    }
}

// Motor number is 1-4 (0-3)
void setMotorIndex(String motorNumber)
{
    try
    {
        uint8_t motorNumberInt = motorNumber.toInt();
        if (motorNumberInt >= NUM_DRIVERS + 1 || motorNumberInt <= 0)
        {
            Serial.printf("ERROR: Invalid motor number! %d\n", motorNumberInt);
            return;
        }
        currentIndex = motorNumberInt - 1;
    }
    catch (const std::exception& e)
    {
        Serial.printf("ERROR: Invalid motor number! %s\n", e.what());
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

    while (1)
    {
        // Show driver connection message only once
        static bool isDriverConnectedMessageShown = false;

        if (!driver[currentIndex]->testConnection() && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.printf("Driver %d connection failed!\n", currentIndex + 1);
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // Reset the message shown flag
        isDriverConnectedMessageShown = false;

        for (int8_t ei = 0; ei < NUM_DRIVERS; ei++)
        {
            if (ei != currentIndex)
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

        encoder[currentIndex]->processPWM();
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

    while (1)
    {
        // Show driver connection message only once
        static bool isDriverConnectedMessageShown = false;

        if (!driver[currentIndex]->testConnection() && !isDriverConnectedMessageShown)
        {
            isDriverConnectedMessageShown = true;
            Serial.printf("Driver %d connection failed!\n", currentIndex + 1);
            esp_task_wdt_reset();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // Reset the message shown flag
        isDriverConnectedMessageShown = false;

        // Show driver status only once
        static bool hasReportedDriverStatus = false;

        if (!hasReportedDriverStatus)
        {
            hasReportedDriverStatus = true;

            auto status = driver[currentIndex]->getDriverStatus();
            Serial.printf("Motor %d Status:\n", currentIndex + 1);
            Serial.printf("  Current: %d mA\n", status.current);
            Serial.printf("  Temperature: %d\n", status.temperature);

            // Get current speed based on motor index
            int16_t speed = motor[currentIndex]->getCurrentSpeed();
            Serial.printf("  Speed: %d\n", speed);
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
                motor[currentIndex]->moveForward();
                esp_task_wdt_reset();
                continue;
            }
            else if (c == 'k')
            {
                motor[currentIndex]->moveBackward();
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
                    String motorNumStr = c.getArgument("n").getValue();
                    setMotorIndex(motorNumStr);
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
                    uint8_t motorNumber = currentIndex + 1;
                    int32_t position    = getMotorPosition();
                    Serial.print(F("*"));
                    Serial.print(motorNumber);
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
                motor[currentIndex]->stopMotor();
                delay(1000);
                Serial.println(F("Restarting..."));
                delay(1000);
                ESP.restart();
            }
            else if (c == cmdStop)
            {
                motor[currentIndex]->stopMotor();
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

void printSerial()
{
    EncoderContext ec = encoder[currentIndex]->getEncoderContext();

    float current_pos = (currentIndex == (uint8_t)MotorType::LINEAR) ? ec.total_travel_um : ec.position_degrees;
    float error       = motor[currentIndex]->calculateSignedPositionError(targetPosition[currentIndex], current_pos);

    if (currentIndex != (uint8_t)MotorType::LINEAR)
        error = motor[currentIndex]->wrapAngle180(error);

    if (fabs(ec.current_pulse - last_pulse[currentIndex]) > 1)
    {
        //  table header
        Serial.print(F("MOT\tDIR\tCPL\tLAP\tPRD\tPDG\tPMM\tTTM\tTTU\tTAR\tERR\n"));

        // Format all values into the buffer
        Serial.print((currentIndex + 1));
        Serial.print(F("\t"));
        Serial.print(ec.direction);
        Serial.print(F("\t"));
        Serial.print(ec.current_pulse);
        Serial.print(F("\t"));
        Serial.print(ec.lap_id);
        Serial.print(F("\t"));
        Serial.print(ec.lap_period);
        Serial.print(F("\t"));
        Serial.print(ec.position_degrees, 2);
        Serial.print(F("\t"));
        Serial.print(ec.position_mm, 3);
        Serial.print(F("\t"));
        Serial.print(ec.total_travel_mm, 2);
        Serial.print(F("\t"));
        Serial.print(ec.total_travel_um, 2);
        Serial.print(F("\t"));
        Serial.print(targetPosition[currentIndex]);
        Serial.print(F("\t"));
        Serial.print(error);
        Serial.println("\n");

        last_pulse[currentIndex] = ec.current_pulse;
    }
}

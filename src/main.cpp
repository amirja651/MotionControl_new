#include "MotorSpeedController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <CLIManager.h>
#include <SPI.h>
#include <esp_task_wdt.h>

TMC5160Manager driver[4] = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]),
                            TMC5160Manager(3, DriverPins::CS[3])};

MotorSpeedController motor[4] = {MotorSpeedController(0, driver[0], DriverPins::DIR[0], DriverPins::STEP[0], DriverPins::EN[0]),
                                 MotorSpeedController(1, driver[1], DriverPins::DIR[1], DriverPins::STEP[1], DriverPins::EN[1]),
                                 MotorSpeedController(2, driver[2], DriverPins::DIR[2], DriverPins::STEP[2], DriverPins::EN[2]),
                                 MotorSpeedController(3, driver[3], DriverPins::DIR[3], DriverPins::STEP[3], DriverPins::EN[3])};

AccelStepper stepper[4] = {
    AccelStepper(AccelStepper::DRIVER, DriverPins::STEP[0], DriverPins::DIR[0]), AccelStepper(AccelStepper::DRIVER, DriverPins::STEP[1], DriverPins::DIR[1]),
    AccelStepper(AccelStepper::DRIVER, DriverPins::STEP[2], DriverPins::DIR[2]), AccelStepper(AccelStepper::DRIVER, DriverPins::STEP[3], DriverPins::DIR[3])};

static constexpr uint32_t SPI_CLOCK            = 1000000;  // 1MHz SPI clock
TaskHandle_t              serialReadTaskHandle = NULL;

// Command history support
#define HISTORY_SIZE 10
static String  commandHistory[HISTORY_SIZE];
static int     historyCount = 0;
static int     historyIndex = -1;  // -1 means not navigating
static uint8_t currentIndex = 1;   // Current driver index

// Step delay control
static uint16_t           stepDelay            = 200;   // Default step delay in microseconds
static constexpr uint16_t MIN_STEP_DELAY       = 1;     // Minimum step delay
static constexpr uint16_t MAX_STEP_DELAY       = 5000;  // Maximum step delay
static constexpr uint16_t STEP_DELAY_INCREMENT = 50;    // Step delay change increment

// Current control parameters
static constexpr uint16_t MIN_RMS_CURRENT = 100;  // Minimum RMS current in mA
static constexpr uint16_t MAX_RMS_CURRENT = 500;  // Maximum RMS current in mA

static uint16_t MICROSTEPS[] = {1, 2, 8, 16, 32, 64, 128, 256};

static constexpr uint16_t MIN_MICROSTEPS = 0;  // Maximum microsteps
static constexpr uint16_t MAX_MICROSTEPS = 7;  // Maximum microsteps

static constexpr uint8_t MIN_IRUN = 1;   // Minimum IRUN value (1/32 of RMS current)
static constexpr uint8_t MAX_IRUN = 31;  // Maximum IRUN value (31/32 of RMS current)

static constexpr uint8_t MIN_IHOLD = 1;   // Minimum IHOLD value (1/32 of RMS current)
static constexpr uint8_t MAX_IHOLD = 31;  // Maximum IHOLD value (31/32 of RMS current)

static constexpr uint16_t CURRENT_INCREMENT = 50;  // Current change increment
static constexpr uint8_t  IRUN_INCREMENT    = 1;   // IRUN change increment
static constexpr uint8_t  IHOLD_INCREMENT   = 1;   // IHOLD change increment

// Movement control parameters
static bool               motorMoving    = false;    // Track if motor is currently moving
static constexpr uint32_t MOVEMENT_STEPS = 1000;     // Number of steps to move
static constexpr float    MOVEMENT_SPEED = 1000.0f;  // Steps per second

static long targetSteps = 0;

int findIndex(uint16_t array[], int length, uint16_t value)
{
    for (int i = 0; i < length; i++)
    {
        if (array[i] == value)
        {
            return i;
        }
    }
    return -1;
}

uint8_t calculateByDesiredCurrent(uint16_t rms_current, uint8_t desired_current)
{
    return (desired_current * 32) / rms_current;
}

uint16_t calculateByNumber(uint16_t rms_current, uint8_t number)
{
    return (rms_current * number) / 32;
}

void printAllSettings()
{
    Serial.println(F("-- Current Settings:"));
    Serial.print(F("RMS current: "));
    Serial.print(driver[currentIndex].getRmsCurrent());
    Serial.print(F(" mA"));

    Serial.print(F("     "));
    Serial.print(F("IRUN: "));
    Serial.print(driver[currentIndex].getIrun());
    Serial.print(F("/32 of RMS current"));
    Serial.print(F(" ("));
    Serial.print(calculateByNumber(driver[currentIndex].getRmsCurrent(), driver[currentIndex].getIrun()));
    Serial.print(F(" mA)"));

    Serial.print(F("     "));
    Serial.print(F("IHOLD: "));
    Serial.print(driver[currentIndex].getIhold());
    Serial.print(F("/32 of RMS current"));
    Serial.print(F(" ("));
    Serial.print(calculateByNumber(driver[currentIndex].getRmsCurrent(), driver[currentIndex].getIhold()));
    Serial.print(F(" mA)"));

    Serial.print(F("     "));
    Serial.print(F("Microsteps: "));
    Serial.print(driver[currentIndex].getMicrosteps());

    Serial.print(F("     "));
    Serial.print(F("Step delay: "));
    Serial.print(stepDelay);
    Serial.println(F(" us"));
}

void printHelp()
{
    Serial.println();
    Serial.println(F("=== Motor Control System Ready ==="));
    Serial.println(F("Keyboard Controls:"));
    Serial.println(F("  Microsteps:"));
    Serial.println(F("    'j' - Increase microsteps"));
    Serial.println(F("    'm' - Decrease microsteps"));
    Serial.println(F("  Step Delay:"));
    Serial.println(F("    'g' - Increase step delay"));
    Serial.println(F("    'b' - Decrease step delay"));
    Serial.println(F("  RMS Current:"));
    Serial.println(F("    'f' - Increase RMS current"));
    Serial.println(F("    'v' - Decrease RMS current"));
    Serial.println(F("  IRUN Current:"));
    Serial.println(F("    'd' - Increase IRUN"));
    Serial.println(F("    'c' - Decrease IRUN"));
    Serial.println(F("  IHOLD Current:"));
    Serial.println(F("    's' - Increase IHOLD"));
    Serial.println(F("    'x' - Decrease IHOLD"));
    Serial.println(F("  Movement Control:"));
    Serial.println(F("    'a' - Start motor movement"));
    Serial.println(F("    'z' - Stop motor movement"));
    printAllSettings();
    Serial.println(F("=================================="));
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
    // Serial.print(F("[Info][M101] Motor is selected: "));
    // Serial.println(index);
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

            else if (c == 'j')  // Increase microsteps
            {
                uint16_t currentMicrosteps = driver[currentIndex].getMicrosteps();

                int index = findIndex(MICROSTEPS, sizeof(MICROSTEPS) / sizeof(MICROSTEPS[0]), currentMicrosteps);
                if (index < MAX_MICROSTEPS)
                {
                    uint16_t newMicrosteps = MICROSTEPS[index + 1];
                    driver[currentIndex].setMicrosteps(newMicrosteps);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'm')  // Decrease microsteps
            {
                uint16_t currentMicrosteps = driver[currentIndex].getMicrosteps();

                int index = findIndex(MICROSTEPS, sizeof(MICROSTEPS) / sizeof(MICROSTEPS[0]), currentMicrosteps);
                if (index > MIN_MICROSTEPS)
                {
                    uint16_t newMicrosteps = MICROSTEPS[index - 1];
                    driver[currentIndex].setMicrosteps(newMicrosteps);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }

            else if (c == 'g')  // Increase step delay
            {
                if (stepDelay < MAX_STEP_DELAY)
                {
                    stepDelay += STEP_DELAY_INCREMENT;
                    printAllSettings();
                }
                // Clear the input buffer since this is a direct command
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'b')  // Decrease step delay
            {
                if (stepDelay > MIN_STEP_DELAY)
                {
                    stepDelay -= STEP_DELAY_INCREMENT;
                    printAllSettings();
                }
                // Clear the input buffer since this is a direct command
                inputBuffer = "";
                lastInput   = "";
            }

            else if (c == 'f')  // Increase RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms < MAX_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms + CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'v')  // Decrease RMS current
            {
                uint16_t currentRms = driver[currentIndex].getRmsCurrent();
                if (currentRms > MIN_RMS_CURRENT)
                {
                    uint16_t newRms = currentRms - CURRENT_INCREMENT;
                    driver[currentIndex].setRmsCurrent(newRms);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }

            else if (c == 'd')  // Increase IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun < MAX_IRUN)
                {
                    uint8_t newIrun = currentIrun + IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'c')  // Decrease IRUN
            {
                uint8_t currentIrun = driver[currentIndex].getIrun();
                if (currentIrun > MIN_IRUN)
                {
                    uint8_t newIrun = currentIrun - IRUN_INCREMENT;
                    driver[currentIndex].setIrun(newIrun);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }

            else if (c == 's')  // Increase IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold < MAX_IHOLD)
                {
                    uint8_t newIhold = currentIhold + IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'x')  // Decrease IHOLD
            {
                uint8_t currentIhold = driver[currentIndex].getIhold();
                if (currentIhold > MIN_IHOLD)
                {
                    uint8_t newIhold = currentIhold - IHOLD_INCREMENT;
                    driver[currentIndex].setIhold(newIhold);
                    printAllSettings();
                }
                inputBuffer = "";
                lastInput   = "";
            }

            else if (c == 'a')  // Start motor movement
            {
                if (!motorMoving)
                {
                    motorMoving = true;
                    motor[currentIndex].enable(true);
                    motor[currentIndex].setDirection(true);  // Forward direction
                    motor[currentIndex].move(MOVEMENT_STEPS, MOVEMENT_SPEED, MOVEMENT_SPEED);
                    motor[currentIndex].attachInterruptHandler();
                    motor[currentIndex].startTimer();
                    Serial.print(F("\r\n[Info] Motor "));
                    Serial.print(currentIndex + 1);
                    Serial.print(F(" started moving forward ("));
                    Serial.print(MOVEMENT_STEPS);
                    Serial.print(F(" steps at "));
                    Serial.print(MOVEMENT_SPEED);
                    Serial.println(F(" steps/sec)\r\n"));
                }
                else
                {
                    Serial.print(F("\r\n[Warning] Motor "));
                    Serial.print(currentIndex + 1);
                    Serial.println(F(" is already moving!\r\n"));
                }
                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'z')  // Stop motor movement
            {
                if (motorMoving)
                {
                    motorMoving = false;
                    motor[currentIndex].stop();
                    motor[currentIndex].stopTimer();
                    motor[currentIndex].detachInterruptHandler();
                    Serial.print(F("\r\n[Info] Motor "));
                    Serial.print(currentIndex + 1);
                    Serial.println(F(" stopped\r\n"));
                }
                else
                {
                    Serial.print(F("\r\n[Warning] Motor "));
                    Serial.print(currentIndex + 1);
                    Serial.println(F(" is not moving!\r\n"));
                }
                inputBuffer = "";
                lastInput   = "";
            }

            else if (c != 'a' && c != 'z' && c != 's' && c != 'x' && c != 'd' && c != 'c' && c != 'f' && c != 'v' && c != 'g' && c != 'b' && c != 'j' &&
                     c != 'm')  // Only add to buffer if not a direct command
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
                if (c.getArgument("i").isSet())
                {
                    String motorIdStr = c.getArgument("i").getValue();
                    setMotorId(motorIdStr);
                }
                if (c.getArgument("p").isSet())
                {
                    String degreesStr  = c.getArgument("p").getValue();
                    int    stepsPerRev = 200 * 256;
                    targetSteps        = (degreesStr.toFloat() / 360.0) * stepsPerRev;

                    stepper[currentIndex].moveTo(targetSteps);
                    stepper[currentIndex].enableOutputs();
                    motorMoving = true;

                    Serial.print(F("Target steps: "));
                    Serial.println(targetSteps);
                    Serial.print(F("Current position: "));
                    Serial.println(stepper[currentIndex].currentPosition());
                }
                else
                {
                    Serial.println(F("ERROR: Motor Id (-n) requires a value"));
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
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
            // Configure motor parameters
            driver[index].configureDriver_All_Motors(true);

            // Initialize all motor controllers
            motor[index].begin();
        }
    }

    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 4096, NULL, 2, &serialReadTaskHandle, 0);
    esp_task_wdt_add(serialReadTaskHandle);  // Register with WDT

    // motor[currentIndex].enable(true);

    static constexpr uint32_t steps_per_mm = 80;
    stepper[currentIndex].setMaxSpeed(50 * steps_per_mm);        // 100mm/s @ 80 steps/mm
    stepper[currentIndex].setAcceleration(1000 * steps_per_mm);  // 2000mm/s^2
    stepper[currentIndex].setEnablePin(DriverPins::EN[2]);
    stepper[currentIndex].setPinsInverted(false, false, true);
    stepper[currentIndex].enableOutputs();
}

void loop()
{
    // Only step manually if motor is not moving (controlled by timer)
    if (motorMoving)
    {
        // motor[currentIndex].motorStep(stepDelay);
        if (stepper[currentIndex].distanceToGo() == 0)
        {
            motorMoving = false;
            stepper[currentIndex].disableOutputs();
            delay(10);
            Serial.print(F("Target steps: "));
            Serial.println(targetSteps);
            Serial.print(F("Current position: "));
            Serial.println(stepper[currentIndex].currentPosition());
        }
        stepper[currentIndex].run();
    }
    esp_task_wdt_reset();
}

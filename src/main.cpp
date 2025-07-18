#include "MAE3Encoder.h"
#include "MotorSpeedController.h"
#include "PositionController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <CLIManager.h>
#include <EEPROM.h>
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

// EEPROM configuration for origin positions
static constexpr uint16_t EEPROM_SIZE           = 512;                // EEPROM size in bytes
static constexpr uint16_t ORIGIN_POSITIONS_ADDR = 0;                  // Starting address for origin positions
static constexpr uint16_t ORIGIN_POSITIONS_SIZE = 4 * sizeof(float);  // 4 motors * float size
static constexpr uint16_t EEPROM_MAGIC_NUMBER   = 0x1234;             // Magic number to verify EEPROM is initialized
static constexpr uint16_t MAGIC_NUMBER_ADDR     = ORIGIN_POSITIONS_ADDR + ORIGIN_POSITIONS_SIZE;

uint16_t calculateByNumber(uint16_t rms_current, uint8_t number)
{
    return (rms_current * number) / 32;
}

void initializeEEPROM()
{
    EEPROM.begin(EEPROM_SIZE);

    // Check if EEPROM is initialized with magic number
    uint16_t storedMagic = EEPROM.readUShort(MAGIC_NUMBER_ADDR);
    if (storedMagic != EEPROM_MAGIC_NUMBER)
    {
        // Reset watchdog timer before EEPROM initialization
        esp_task_wdt_reset();

        // Initialize EEPROM with default values
        float defaultOrigins[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        for (uint8_t i = 0; i < 4; i++)
        {
            uint16_t addr       = ORIGIN_POSITIONS_ADDR + (i * sizeof(float));
            uint8_t* floatBytes = (uint8_t*)&defaultOrigins[i];

            // Write the float value byte by byte to avoid cache issues
            for (int j = 0; j < sizeof(float); j++)
            {
                EEPROM.write(addr + j, floatBytes[j]);
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(1));  // Give other tasks a chance to run
            }
        }

        // Write magic number byte by byte
        uint8_t* magicBytes = (uint8_t*)&EEPROM_MAGIC_NUMBER;
        for (int i = 0; i < sizeof(uint16_t); i++)
        {
            EEPROM.write(MAGIC_NUMBER_ADDR + i, magicBytes[i]);
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1));  // Give other tasks a chance to run
        }

        // Reset watchdog timer before commit
        esp_task_wdt_reset();

        // Commit with error checking
        bool commitSuccess = EEPROM.commit();

        // Reset watchdog timer after EEPROM operation
        esp_task_wdt_reset();

        if (commitSuccess)
        {
            Serial.println(F("[EEPROM] Initialized with default origin positions"));
        }
        else
        {
            Serial.println(F("[EEPROM] Error: Failed to initialize EEPROM"));
        }
    }
    else
    {
        Serial.println(F("[EEPROM] Already initialized, loading stored origin positions"));
    }
}

void storeOriginPosition(uint8_t motorIndex, float originDegrees)
{
    if (motorIndex >= 4)
    {
        Serial.println(F("[EEPROM] Error: Invalid motor index"));
        return;
    }

    esp_task_wdt_reset();

    uint16_t addr = ORIGIN_POSITIONS_ADDR + (motorIndex * sizeof(float));

    // Write the float value byte by byte to avoid cache issues
    uint8_t* floatBytes = (uint8_t*)&originDegrees;
    for (int i = 0; i < sizeof(float); i++)
    {
        EEPROM.write(addr + i, floatBytes[i]);
        esp_task_wdt_reset();
        delay(1);  // Small delay to prevent watchdog timeout
    }

    esp_task_wdt_reset();
    yield();  // Give other tasks a chance to run
    bool commitSuccess = EEPROM.commit();
    esp_task_wdt_reset();

    if (!commitSuccess)
    {
        Serial.println(F("[EEPROM] Error: Failed to commit to EEPROM"));
    }
}

float loadOriginPosition(uint8_t motorIndex)
{
    if (motorIndex >= 4)
    {
        Serial.println(F("[EEPROM] Error: Invalid motor index, using 0.0"));
        return 0.0f;
    }

    uint16_t addr = ORIGIN_POSITIONS_ADDR + (motorIndex * sizeof(float));

    // Read the float value byte by byte to avoid cache issues
    uint8_t floatBytes[sizeof(float)];
    for (int i = 0; i < sizeof(float); i++)
    {
        floatBytes[i] = EEPROM.read(addr + i);
    }
    float originDegrees = *(float*)floatBytes;

    Serial.print(F("[EEPROM] Loaded origin position for motor "));
    Serial.print(motorIndex + 1);
    Serial.print(F(": "));
    Serial.print(originDegrees, 2);
    Serial.println(F(" degrees"));

    return originDegrees;
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

    Serial.println(F("  Position Control: 'Q' Move to origin "));
    Serial.println(F("  Position Status:  'L' Show position status"));
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

                bool success = positionController[currentIndex].moveToAngle(targetAngle, movementType);

                if (success)
                {
                    Serial.print(F("[Info] Motor "));
                    Serial.print(currentIndex + 1);
                    Serial.print(F(" moving to "));
                    Serial.print(targetAngle, 2);
                    Serial.println(F(" degrees"));
                }
                else
                {
                    Serial.println(F("[Error] Failed to queue movement command"));
                }

                inputBuffer = "";
                lastInput   = "";
            }
            else if (c == 'L')  // Show position status
            {
                MotorStatus status = positionController[currentIndex].getStatus();
                Serial.print(F("\r\n[Position Status] Motor "));
                Serial.print(currentIndex + 1);
                Serial.print(F(": Current: "));
                Serial.print(status.currentAngle, 2);
                Serial.print(F("°, Target: "));
                Serial.print(status.targetAngle, 2);
                Serial.print(F("°, Moving: "));
                Serial.print(status.isMoving ? F("YES") : F("NO"));
                Serial.print(F(", Enabled: "));
                Serial.print(status.isEnabled ? F("YES") : F("NO"));



                if (currentIndex >= 4 || !driverEnabled[currentIndex] || !encoder[currentIndex].isEnabled())
                {
                    Serial.println(F("[Encoder] Error: Invalid motor index or encoder not enabled"));
                    return;
                }

                for (int i = 0; i < 10; i++)
                {
                    encoder[currentIndex].processPWM();
                    delay(100);
                    esp_task_wdt_reset();
                }

                EncoderState encoderState = encoder[currentIndex].getState();
                Serial.print(F(", Encoder: "));
                Serial.print(encoderState.position_degrees, 2);
                Serial.print(F("°"));
                Serial.print(encoderState.direction == Direction::CLOCKWISE ? F(" CW") : F(" CCW"));
                /*Serial.print(F(" (position: "));
                Serial.print(encoderState.position_pulse);
                Serial.print(F(", High: "));
                Serial.print(encoderState.width_high);
                Serial.print(F(", Low: "));
                Serial.print(encoderState.width_low);
                Serial.print(F(", Interval: "));
                Serial.print(encoderState.width_interval);
                Serial.print(F(" pulses)"));*/

                Serial.println();

                inputBuffer = "";
                lastInput   = "";
            }
            else if (c != 'A' && c != 'Z' && c != 'S' && c != 'X' && c != 'D' && c != 'C' && c != 'Q' && c != 'L')  // Only add to buffer if not a direct command
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

                    bool success = positionController[currentIndex].moveToAngle(targetAngle, movementType);

                    if (success)
                    {
                        Serial.print(F("[Info] Motor "));
                        Serial.print(currentIndex + 1);
                        Serial.print(F(" moving to "));
                        Serial.print(targetAngle, 2);
                        Serial.println(F(" degrees"));
                    }
                    else
                    {
                        Serial.println(F("[Error] Failed to queue movement command"));
                    }
                }
                if (c.getArgument("o").isSet())
                {
                    String valueStr = c.getArgument("o").getValue();

                    if (valueStr != "-9999.0")
                    {
                        float                    value = valueStr.toFloat();
                        ConvertValuesFromDegrees cvfd  = positionController[currentIndex].convertFromDegrees(value);
                        ConvertValuesFromPulses  cvfp  = positionController[currentIndex].convertFromPulses(value);
                        ConvertValuesFromSteps   cvfs  = positionController[currentIndex].convertFromMSteps(value);

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
                        positionController[currentIndex].setCurrentPosition(cvfd.STEPS_FROM_DEGREES);

                        // Store origin position in EEPROM
                        storeOriginPosition(currentIndex, value);
                    }
                    else
                    {
                        loadOriginPosition(currentIndex);
                    }
                }
                if (c.getArgument("c").isSet())
                {
                    float position = positionController[currentIndex].getCurrentAngle();
                    Serial.print(F("*"));
                    Serial.print(currentIndex);
                    Serial.print(F("#"));
                    Serial.print(position, 2);
                    Serial.println(F("#"));
                }
                if (c.getArgument("d").isSet())
                {
                    positionController[currentIndex].stop();
                    positionController[currentIndex].disable();
                }
                if (c.getArgument("e").isSet())
                {
                    positionController[currentIndex].enable();
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
            Serial.print(F("ERROR: "));
            Serial.print(cmdError);
            Serial.println();
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

    esp_task_wdt_init(30, true);  // Increased timeout to 30 seconds
    esp_task_wdt_add(NULL);       // Add the current task (setup)
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

    // Initialize EEPROM
    initializeEEPROM();

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

    // Create serial read task
    xTaskCreatePinnedToCore(serialReadTask, "SerialReadTask", 4096, NULL, 2, &serialReadTaskHandle, 0);
    esp_task_wdt_add(serialReadTaskHandle);  // Register with WDT

    setMotorId("2");

    Serial.println(F("[Info] Position control system initialized"));
    Serial.println(F("[Info] Use 'L' to show position status"));
}

void loop()
{
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
}

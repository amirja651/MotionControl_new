#pragma region defin_code
#include "CommandHistory.h"
#include "DirMultiplexer.h"
#include "MAE3Encoder.h"
#include "PositionController.h"
#include "SystemDiagnostics.h"
#include "TMC5160Manager.h"
#include "UnitConverter.h"
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
        int32_t     turns          = 0;
        int32_t     turnsValue     = 0;
        float       value          = 0.0f;
        bool        save           = false;
    } position, voltageDropPosition, orgin, control;
} data;

VoltageMonitor voltageMonitor(VoltageMonitorPins::POWER_3_3, VoltageMonitor::MonitorMode::DIGITAL_, 0, 10);

// Driver status tracking
static bool    driverEnabled[4] = {false, false, false, false};
TMC5160Manager driver[4]        = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

// Direction multiplexer for stepper motor direction control
DirMultiplexer dirMultiplexer(MultiplexerPins::S0, MultiplexerPins::S1, MultiplexerPins::DIR);

// MAE3 Encoders for position feedback (read-only, not used for control)
MAE3Encoder encoder[4] = {MAE3Encoder(EncoderPins::SIGNAL[0], 0), MAE3Encoder(EncoderPins::SIGNAL[1], 1), MAE3Encoder(EncoderPins::SIGNAL[2], 2), MAE3Encoder(EncoderPins::SIGNAL[3], 3)};

// Position controllers for precise movement control
PositionController positionController[4] = {PositionController(0, driver[0], dirMultiplexer, DriverPins::STEP[0], DriverPins::EN[0], encoder[0]), PositionController(1, driver[1], dirMultiplexer, DriverPins::STEP[1], DriverPins::EN[1], encoder[1]),
                                            PositionController(2, driver[2], dirMultiplexer, DriverPins::STEP[2], DriverPins::EN[2], encoder[2]), PositionController(3, driver[3], dirMultiplexer, DriverPins::STEP[3], DriverPins::EN[3], encoder[3])};

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
#pragma endregion

#pragma region function_declarations
// Function declarations
uint16_t     calculateByNumber(uint16_t rms_current, uint8_t number);
void         storeToMemory();
float        loadOriginPosition();
void         loadControlMode();
void         loadPosition();
void         clearAllOriginPositions();
void         printAllOriginPositions();
void         clearLine();
void         setMotorId(String motorId);
void         serialReadTask(void* pvParameters);
float        setCurrentPositionFromPulses();
void         checkDifferenceCorrection();
void         onVoltageDrop();
void         linearProcess(float targetUMeters);
void         rotationalProcess(float targetAngle);
void         prepaireBeforeMovement();
bool         isLinearMotor(uint8_t index);
int32_t      getEncoderSteps();
int32_t      getCurrentSteps();
int32_t      getTargetSteps();
void         handleInAbsenceOfInterrupt();
void         handleMovementComplete();
EncoderState getEncoderState();
MotorStatus  getMotorStatus();
int32_t      getCurrentTurnFromStepper();
void         setDirection(bool fwd);
void         disable();
void         enable();
void         stop();
bool         moveToSteps(int32_t targetSteps, MovementType movementType, ControlMode controlMode);
void         setCurrentPosition(int32_t positionSteps);
void         showPositionStatus();
#pragma endregion

#pragma region setup_and_loop
void setup()
{
    // Initialize SPI
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(115200);

    esp_log_level_set("*", ESP_LOG_INFO);

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

    // Initialize Preferences for storing motor origin positions
    if (!prefs.begin("motion_control", false))  // false = read/write mode
    {
        log_e("Failed to initialize Preferences");
    }
    else
    {
        log_d("Preferences successfully initialized");
    }

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
    handleInAbsenceOfInterrupt();
    handleMovementComplete();
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
#pragma endregion

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

        if (isLinearMotor(currentIndex))
        {
            String key2 = "m" + String(currentIndex) + "_tu";
            log_d("Key: %s, Value: %d", key2.c_str(), data.orgin.turns);
            bool success = prefs.putInt(key2.c_str(), data.orgin.turns);

            if (!success)
            {
                log_e("Failed to store turns for motor %d", currentIndex + 1);
            }
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
        String key1                   = "m" + String(currentIndex) + "_po";
        log_d("Key: %s, Value: %f", key1.c_str(), data.voltageDropPosition.beforeMovement);
        bool success = prefs.putFloat(key1.c_str(), data.voltageDropPosition.beforeMovement);

        if (!success)
        {
            log_e("Failed to store voltage drop for motor %d", currentIndex + 1);
        }

        if (isLinearMotor(currentIndex))
        {
            String key2 = "m" + String(currentIndex) + "_tu";
            log_d("Key: %s, Value: %d", key2.c_str(), data.voltageDropPosition.turns);
            bool success = prefs.putInt(key2.c_str(), data.voltageDropPosition.turns);

            if (!success)
            {
                log_e("Failed to store turns for motor %d", currentIndex + 1);
            }
        }
    }
    if (data.position.save)
    {
        data.position.save = false;
        String key1        = "m" + String(currentIndex) + "_po";
        log_d("Key: %s, Value: %f", key1.c_str(), data.position.reached);
        bool success = prefs.putFloat(key1.c_str(), data.position.reached);

        if (!success)
        {
            log_e("Failed to store target reached for motor %d", currentIndex + 1);
        }

        if (isLinearMotor(currentIndex))
        {
            String key2 = "m" + String(currentIndex) + "_tu";
            log_d("Key: %s, Value: %d", key2.c_str(), data.position.turns);
            bool success = prefs.putInt(key2.c_str(), data.position.turns);

            if (!success)
            {
                log_e("Failed to store turns for motor %d", currentIndex + 1);
            }
        }
    }

    interrupts();
    encoder[currentIndex].setInAbsenceOfInterruptFlag(false);
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
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return;
    }

    String key        = "m" + String(currentIndex) + "_cm";
    data.control.mode = static_cast<ControlMode>(prefs.getInt(key.c_str(), static_cast<int>(ControlMode::HYBRID)));
    log_d("Motor %d control mode loaded: %d", currentIndex + 1, static_cast<int>(data.control.mode));
}

void loadPosition()
{
    if (currentIndex >= 4)
    {
        log_e("Invalid motor index!");
        return;
    }

    String key1         = "m" + String(currentIndex) + "_po";
    data.position.value = prefs.getFloat(key1.c_str(), 0.0f);

    if (isLinearMotor(currentIndex))
    {
        String key2         = "m" + String(currentIndex) + "_tu";
        data.position.turns = prefs.getInt(key2.c_str(), 0);
        log_d("Motor %d position loaded: Position %f, Turns %d", currentIndex + 1, data.position.value, data.position.turns);
    }
    else
    {
        log_d("Motor %d position loaded: Position %f", currentIndex + 1, data.position.value);
    }
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
        String key            = "m" + String(i) + "_or";
        float  originPosition = prefs.getFloat(key.c_str(), 0.0f);
        log_d("Motor %d: %f°", i + 1, originPosition);

        // Small delay between each motor to prevent overload
        delay(5);

        if (isLinearMotor(i))
        {
            String  key2  = "m" + String(i) + "_tu";
            int32_t turns = prefs.getInt(key2.c_str(), 0);
            log_d("Motor %d: %d turns", i + 1, turns);

            if (turns != 0)
            {
                originPosition = turns * PIXEL_SIZE_MM;
                log_d("Motor %d: Origin Position %f°", i + 1, originPosition);
            }
        }
    }

    // Print total number of keys for debugging
    size_t totalKeys = prefs.freeEntries();
    log_d("Total free entries: %d", totalKeys);
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
    log_i("Motor %d selected and enabled", currentIndex + 1);
}

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
            else if (c == commands[static_cast<int>(CommandKey::Q)][0])  // 'Q' go to position from memory
            {
                float target = data.position.value;

                if (isLinearMotor(currentIndex))
                {
                    linearProcess(target);
                }
                else
                {
                    rotationalProcess(target);
                }
            }
            else if (c == commands[static_cast<int>(CommandKey::L)][0])  // 'L' Show position status
            {
                showPositionStatus();
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
                        uint32_t pulses       = getEncoderState().position_pulse;
                        float    encoderAngle = UnitConverter::convertFromPulses(pulses).TO_DEGREES;
                        Serial.print(F("  Encoder Position: "));
                        Serial.print(encoderAngle);
                        Serial.print(F("° ("));
                        Serial.print(pulses);
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
                            setDirection(true);
                        }
                        else if (motorIdStr == "2")
                        {
                            currentIndex = 1;
                            setDirection(true);
                        }
                        else if (motorIdStr == "3")
                        {
                            currentIndex = 2;
                            setDirection(true);
                        }
                        else if (motorIdStr == "4")
                        {
                            currentIndex = 3;
                            setDirection(true);
                        }
                        else if (motorIdStr == "5")
                        {
                            currentIndex = 0;
                            setDirection(false);
                        }
                        else if (motorIdStr == "6")
                        {
                            currentIndex = 1;
                            setDirection(false);
                        }
                        else if (motorIdStr == "7")
                        {
                            currentIndex = 2;
                            setDirection(false);
                        }
                        else if (motorIdStr == "8")
                        {
                            currentIndex = 3;
                            setDirection(false);
                        }
                    }
                    setMotorId(motorIdStr);
                }
            }
            else if (c == cmdMove)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                if (c.getArgument("p").isSet())
                {
                    String targetStr = c.getArgument("p").getValue();
                    float  target    = targetStr.toFloat();

                    if (isLinearMotor(currentIndex))
                    {
                        linearProcess(target);
                    }
                    else
                    {
                        rotationalProcess(target);
                    }
                }
            }
            else if (c == cmdControlMode)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                if (c.getArgument("o").isSet())
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
            else if (c == cmdStop)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                stop();
            }
            else if (c == cmdEnable)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                enable();
            }
            else if (c == cmdDisable)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                stop();
                vTaskDelay(pdMS_TO_TICKS(100));
                disable();
            }
            else if (c == cmdCurrentPosition)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                int32_t currentSteps = getCurrentSteps();
                float   position     = UnitConverter::convertFromSteps(currentSteps).TO_DEGREES;
                Serial.print(F("CURRENT"));
                Serial.print(position);
            }
            else if (c == cmdLastPosition)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                }
                float targetAngle = data.position.value;
                Serial.print(F("LAST"));
                Serial.print(targetAngle);
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
                        float value      = setCurrentPositionFromPulses();
                        data.orgin.value = value;
                        data.orgin.save  = true;

                        if (1)
                        {
                            ConvertValues::FromDegrees cvfd = UnitConverter::convertFromDegrees(value);
                            ConvertValues::FromPulses  cvfp = UnitConverter::convertFromPulses(value);
                            ConvertValues::FromSteps   cvfs = UnitConverter::convertFromSteps(value);
                            Serial.print(F("From Degrees: "));
                            Serial.print(cvfd.TO_PULSES);
                            Serial.print(F(", "));
                            Serial.println(cvfd.TO_STEPS);
                            Serial.print(F("From Pulses: "));
                            Serial.print(cvfp.TO_DEGREES);
                            Serial.print(F(", "));
                            Serial.println(cvfp.TO_STEPS);
                            Serial.print(F("From Steps: "));
                            Serial.print(cvfs.TO_DEGREES);
                            Serial.print(F(", "));
                            Serial.println(cvfs.TO_PULSES);
                        }
                    }
                }
            }
            else if (c == cmdRestart)
            {
                ESP.restart();
            }
            else if (c == cmdShow)
            {
                if (c.getArgument("n").isSet())
                {
                    String motorIdStr = c.getArgument("n").getValue();
                    setMotorId(motorIdStr);
                    showPositionStatus();
                }
            }
            else if (c == cmdHelp)
            {
                Serial.print(F("Help:"));
                Serial.print(cli.toString());
                Serial.println();
            }
            else if (c == cmdTest)
            {
                if (c.getArgument("p").isSet())
                {
                    String                     valueStr = c.getArgument("p").getValue();
                    float                      value    = valueStr.toFloat();
                    ConvertValues::FromDegrees cvfd     = UnitConverter::convertFromDegrees(value);
                    ConvertValues::FromPulses  cvfp     = UnitConverter::convertFromPulses(value);
                    ConvertValues::FromSteps   cvfs     = UnitConverter::convertFromSteps(value);
                    ConvertValues::FromUMeters cvfm     = UnitConverter::convertFromUMeters(value);

                    Serial.print(F("From Degrees: Pulses:"));
                    Serial.print(cvfd.TO_PULSES);
                    Serial.print(F(", Steps:"));
                    Serial.print(cvfd.TO_STEPS);
                    Serial.print(F(", Turns:"));
                    Serial.print(cvfd.TO_TURNS);
                    Serial.print(F(", Micrometers:"));
                    Serial.println(cvfd.TO_UMETERS);

                    Serial.print(F("From Pulses: Degrees:"));
                    Serial.print(cvfp.TO_DEGREES);
                    Serial.print(F(", Steps:"));
                    Serial.print(cvfp.TO_STEPS);
                    Serial.print(F(", Turns:"));
                    Serial.print(cvfp.TO_TURNS);
                    Serial.print(F(", Micrometers:"));
                    Serial.println(cvfp.TO_UMETERS);

                    Serial.print(F("From Steps: Degrees:"));
                    Serial.print(cvfs.TO_DEGREES);
                    Serial.print(F(", Pulses:"));
                    Serial.print(cvfs.TO_PULSES);
                    Serial.print(F(", Turns:"));
                    Serial.print(cvfs.TO_TURNS);
                    Serial.print(F(", Micrometers:"));
                    Serial.println(cvfs.TO_UMETERS);

                    Serial.print(F("From Micrometers: Degrees:"));
                    Serial.print(cvfm.TO_DEGREES);
                    Serial.print(F(", Steps:"));
                    Serial.print(cvfm.TO_STEPS);
                    Serial.print(F(", Pulses:"));
                    Serial.print(cvfm.TO_PULSES);
                    Serial.print(F(", Turns:"));
                    Serial.println(cvfm.TO_TURNS);
                    Serial.println();
                }
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

float setCurrentPositionFromPulses()
{
    uint32_t pulses = getEncoderState().position_pulse;
    int32_t  steps  = UnitConverter::convertFromPulses(pulses).TO_STEPS;
    setCurrentPosition(steps);
    float output;
    if (isLinearMotor(currentIndex))
    {
        float   micrometers = UnitConverter::convertFromPulses(pulses).TO_UMETERS;
        int32_t turns       = UnitConverter::convertFromPulses(pulses).TO_TURNS;
        output              = micrometers;
        log_d("Motor %d current position set to Steps: %d (%f µm, %d t)", currentIndex + 1, steps, micrometers, turns);
    }
    else
    {
        float angle = UnitConverter::convertFromPulses(pulses).TO_DEGREES;
        output      = angle;
        log_d("Motor %d current position set to Steps: %d (%f°)", currentIndex + 1, steps, angle);
    }
    return output;
}

void checkDifferenceCorrection()
{
    encoder[currentIndex].processPWM();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_task_wdt_reset();
    int32_t encoderSteps = getEncoderSteps();
    int32_t currentSteps = getCurrentSteps();
    int32_t targetSteps  = getTargetSteps();

    float encoderAngle = UnitConverter::convertFromSteps(encoderSteps).TO_DEGREES;
    float currentAngle = UnitConverter::convertFromSteps(currentSteps).TO_DEGREES;
    float targetAngle  = UnitConverter::convertFromSteps(targetSteps).TO_DEGREES;
    //  Serial.printf("Encoder: %f, Current: %f, Target: %f\n", encoderAngle, currentAngle, targetAngle);

    float difference = encoderAngle - currentAngle;

    if (data.control.mode == ControlMode::OPEN_LOOP && fabs(difference) > 0.05)
    {
        setCurrentPosition(encoderSteps);

        if (targetAngle == 0)
            targetAngle = 0.01;
        else if (targetAngle == 360)
            targetAngle = 359.9955f;

        // encoder[currentIndex].attachOnComplete(storeOriginPosition);
        positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);

        int32_t targetSteps = UnitConverter::convertFromDegrees(targetAngle).TO_STEPS;
        bool    success     = moveToSteps(targetSteps, MovementType::SHORT_RANGE, data.control.mode);

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

        if (isLinearMotor(currentIndex))
        {
            data.position.turns = getCurrentTurnFromStepper();
        }

        data.position.save = true;
        storeToMemory();
        log_i("No movement needed, difference: %f, ControlMode: %s", difference, (data.control.mode == ControlMode::OPEN_LOOP) ? "open-loop" : "hybrid");
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
        stop();
        data.voltageDropPosition.save = true;
        storeToMemory();

        log_w("Motor %d stopped due to voltage drop, position saved", currentIndex + 1);
    }
}

void linearProcess(float targetUMeters)
{
    data.voltageDropPosition.turns = getCurrentTurnFromStepper();
    prepaireBeforeMovement();

    int32_t targetSteps = UnitConverter::convertFromUMeters(targetUMeters).TO_STEPS;
    bool    success     = moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, data.control.mode);

    if (success)
    {
        const char* modeStr = (data.control.mode == ControlMode::OPEN_LOOP) ? "open-loop" : "hybrid";
        log_i("Motor %d moving to %f µm (%s)", currentIndex + 1, targetUMeters, modeStr);
    }
    else
    {
        log_e("Failed to queue movement command");
    }
}

void rotationalProcess(float targetAngle)
{
    if (targetAngle == 0)
        targetAngle = 0.09f;
    else if (targetAngle == 360)
        targetAngle = 359.9955f;

    data.voltageDropPosition.turns = 0;
    prepaireBeforeMovement();

    int32_t targetSteps = UnitConverter::convertFromDegrees(targetAngle).TO_STEPS;
    bool    success     = moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, data.control.mode);

    if (success)
    {
        const char* modeStr = (data.control.mode == ControlMode::OPEN_LOOP) ? "open-loop" : "hybrid";
        log_i("Motor %d moving to %f degrees (%s)", currentIndex + 1, targetAngle, modeStr);
    }
    else
    {
        log_e("Failed to queue movement command");
    }
}

void prepaireBeforeMovement()
{
    data.voltageDropPosition.beforeMovement = setCurrentPositionFromPulses();
    Serial.print(F("Before:"));
    Serial.print(data.voltageDropPosition.beforeMovement);
    Serial.print(F("Turns:"));
    Serial.println(data.voltageDropPosition.turns);
    loadControlMode();
    encoder[currentIndex].attachInAbsenceOfInterrupt(storeToMemory);
    positionController[currentIndex].attachOnComplete(checkDifferenceCorrection);
}

#pragma region helper_functions
int32_t getEncoderSteps()
{
    return positionController[currentIndex].getEncoderSteps();
}
int32_t getCurrentSteps()
{
    return positionController[currentIndex].getCurrentSteps();
}
int32_t getTargetSteps()
{
    return positionController[currentIndex].getTargetSteps();
}
bool isLinearMotor(uint8_t index)
{
    return (positionController[index].getMotorType() == MotorType::LINEAR);
}
void handleInAbsenceOfInterrupt()
{
    encoder[currentIndex].handleInAbsenceOfInterrupt();
}
void handleMovementComplete()
{
    positionController[currentIndex].handleMovementComplete();
}
EncoderState getEncoderState()
{
    return positionController[currentIndex].getEncoderState();
}
MotorStatus getMotorStatus()
{
    return positionController[currentIndex].getMotorStatus();
}
int32_t getCurrentTurnFromStepper()
{
    return positionController[currentIndex].getCurrentTurnFromStepper();
}
void setDirection(bool fwd)
{
    positionController[currentIndex].setDirection(fwd);
}
void disable()
{
    positionController[currentIndex].disable();
}
void enable()
{
    positionController[currentIndex].enable();
}
void stop()
{
    positionController[currentIndex].stop();
}
bool moveToSteps(int32_t targetSteps, MovementType movementType, ControlMode controlMode)
{
    return positionController[currentIndex].moveToSteps(targetSteps, MovementType::MEDIUM_RANGE, data.control.mode);
}
void setCurrentPosition(int32_t positionSteps)
{
    positionController[currentIndex].setCurrentPosition(positionSteps);
}
void showPositionStatus()
{
    uint32_t    pulses      = getEncoderState().position_pulse;
    Direction   direction   = getEncoderState().direction;
    MotorStatus motorStatus = getMotorStatus();

    if (motorStatus.currentSteps == 0)
    {
        setCurrentPositionFromPulses();
        motorStatus.currentSteps = getCurrentSteps();
    }

    // Check if this is a linear motor (motor 1) or rotational motor
    if (isLinearMotor(currentIndex))
    {
        // For linear motor, display in micrometers
        float   encoderMicrometers     = UnitConverter::convertFromPulses(pulses).TO_UMETERS;
        float   currentMicrometers     = UnitConverter::convertFromSteps(motorStatus.currentSteps).TO_UMETERS;
        int32_t turns                  = UnitConverter::convertFromSteps(motorStatus.currentSteps).TO_TURNS;
        float   targetMicrometers      = UnitConverter::convertFromSteps(motorStatus.targetSteps).TO_UMETERS;
        float   diff                   = currentMicrometers - encoderMicrometers;
        data.voltageDropPosition.turns = turns;

        Serial.print(F("[Position Status] Motor "));
        Serial.print(currentIndex + 1);
        Serial.print(F(" (Linear): Diff="));
        Serial.print(diff);
        Serial.print(F(" µm, Current="));
        Serial.print(currentMicrometers);
        Serial.print(F(" µm 💡, Turns="));
        Serial.print(turns);
        Serial.print(F(", Target:"));
        Serial.print(targetMicrometers);
        Serial.print(F(" µm, Moving="));
        Serial.print(motorStatus.isMoving ? F("YES") : F("NO"));
        Serial.print(F(", Enabled="));
        Serial.print(motorStatus.isEnabled ? F("YES") : F("NO"));
        Serial.print(F(", Mode="));
        const char* modeStr = (motorStatus.controlMode == ControlMode::OPEN_LOOP) ? "OPEN L" : "HYBRID";
        Serial.print(modeStr);
        Serial.print(F(", (Encoder: "));
        Serial.print(encoderMicrometers);
        Serial.print(F(" µm,"));
        Serial.print(direction == Direction::CLOCKWISE ? F(" CW") : F(" CCW"));
        Serial.print(F(", "));
        Serial.print(pulses);
        Serial.println(F(" pulses)"));
    }
    else
    {
        // For rotational motors, display in degrees
        float encoderAngle = UnitConverter::convertFromPulses(pulses).TO_DEGREES;
        float currentAngle = UnitConverter::convertFromSteps(motorStatus.currentSteps).TO_DEGREES;
        float targetAngle  = UnitConverter::convertFromSteps(motorStatus.targetSteps).TO_DEGREES;
        float diff         = currentAngle - encoderAngle;

        Serial.print(F("[Position Status] Motor "));
        Serial.print(currentIndex + 1);
        Serial.print(F(" (Rotational): Diff="));
        Serial.print(diff);
        Serial.print(F("°, Current="));
        Serial.print(currentAngle);
        Serial.print(F("° 💡, Target="));
        Serial.print(targetAngle);
        Serial.print(F("°, Moving="));
        Serial.print(motorStatus.isMoving ? F("YES") : F("NO"));
        Serial.print(F(", Enabled="));
        Serial.print(motorStatus.isEnabled ? F("YES") : F("NO"));
        Serial.print(F(", Mode="));
        const char* modeStr = (motorStatus.controlMode == ControlMode::OPEN_LOOP) ? "OPEN L" : "HYBRID";
        Serial.print(modeStr);
        Serial.print(F(", (Encoder: "));
        Serial.print(encoderAngle);
        Serial.print(F("°,"));
        Serial.print(direction == Direction::CLOCKWISE ? F(" CW") : F(" CCW"));
        Serial.print(F(", "));
        Serial.print(pulses);
        Serial.println(F(" pulses)"));
    }
}
#pragma endregion
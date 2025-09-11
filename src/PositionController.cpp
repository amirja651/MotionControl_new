#include "PositionController.h"

#include <esp_task_wdt.h>

#include <cmath>

// Static member initialization
TaskHandle_t        PositionController::_positionControlTaskHandle = nullptr;
QueueHandle_t       PositionController::_movementCommandQueue      = nullptr;
SemaphoreHandle_t   PositionController::_statusMutex               = nullptr;
PositionController* PositionController::_instances[4]              = {nullptr};

// Constructor with encoder mae3
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin, mae3::Mae3Encoder& encoderMae3)
    : _driver(driver),
      _stepper(AccelStepper::DRIVER,
               stepPin,
               0),  // dirPin will be handled by multiplexer
      _encoderMae3(&encoderMae3),
      _dirMultiplexer(dirMultiplexer),
      _stepPin(stepPin),
      _enPin(enPin),
      _motorId(motorId),
      _status{},
      _enabled(false),
      _initialized(false),
      _controlMode(ControlMode::OPEN_LOOP),
      _movementCompleteFlag(false),
      _onComplete(nullptr)
{
    // Initialize status
    _status.motorId            = motorId;
    _status.currentSteps       = 0;
    _status.targetSteps        = 0;
    _status.isMoving           = false;
    _status.isEnabled          = false;
    _status.lastMovementType   = MovementType::MEDIUM_RANGE;
    _status.movementStartTime  = 0;
    _status.totalMovementTime  = 0;
    _status.controlMode        = ControlMode::OPEN_LOOP;
    _status.positionErrorSteps = 0;
    _status.encoderSteps       = 0;

    // Store instance for RTOS access
    _instances[motorId] = this;

    // Initialize speed profiles with default values
    _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {2000.0f, 4000.0f};   // Increased from 1000
    _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {4000.0f, 8000.0f};   // Increased from 2000
    _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {8000.0f, 16000.0f};  // Increased from 4000

    // Initialize distance-based speed profiles for precise control
    // NEGLIGIBLE (< 6 steps) - Not used, but defined for completeness
    _distanceSpeedProfiles[static_cast<int>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f, 0};

    // VERY_SHORT (6 - 32 steps) - Very slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f, 6};

    // SHORT (32 - 64 steps) - Slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::SHORT)] = {1000.0f, 2000.0f, 32};

    // MEDIUM (64 - 320 steps) - Balanced speed and precision
    _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)] = {2000.0f, 4000.0f, 64};

    // LONG (320 - 640 steps) - Moderate speed
    _distanceSpeedProfiles[static_cast<int>(DistanceType::LONG)] = {4000.0f, 8000.0f, 320};

    // VERY_LONG (> 640 steps) - Higher speed for longer distances
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_LONG)] = {8000.0f, 16000.0f, 640};
}

// Constructor without encoder (for demo)
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin)
    : _driver(driver),
      _stepper(AccelStepper::DRIVER,
               stepPin,
               0),  // dirPin will be handled by multiplexer
      _encoderMae3(nullptr),
      _dirMultiplexer(dirMultiplexer),
      _stepPin(stepPin),
      _enPin(enPin),
      _motorId(motorId),
      _status{},
      _enabled(false),
      _initialized(false),
      _controlMode(ControlMode::OPEN_LOOP),
      _movementCompleteFlag(false),
      _onComplete(nullptr)
{
    // Initialize status
    _status.motorId           = motorId;
    _status.currentSteps      = 0;
    _status.targetSteps       = 0;
    _status.isMoving          = false;
    _status.isEnabled         = false;
    _status.lastMovementType  = MovementType::MEDIUM_RANGE;
    _status.movementStartTime = 0;
    _status.totalMovementTime = 0;

    // Store instance for RTOS access
    _instances[motorId] = this;

    // Initialize speed profiles with default values
    _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {2000.0f, 4000.0f};   // Increased from 1000
    _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {4000.0f, 8000.0f};   // Increased from 2000
    _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {8000.0f, 16000.0f};  // Increased from 4000

    // Initialize distance-based speed profiles for precise control
    // NEGLIGIBLE (< 6 steps) - Not used, but defined for completeness
    _distanceSpeedProfiles[static_cast<int>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f, 0};

    // VERY_SHORT (6 - 32 steps) - Very slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f, 6};

    // SHORT (32 - 64 steps) - Slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::SHORT)] = {1000.0f, 2000.0f, 32};

    // MEDIUM (64 - 320 steps) - Balanced speed and precision
    _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)] = {2000.0f, 4000.0f, 64};

    // LONG (320 - 640 steps) - Moderate speed
    _distanceSpeedProfiles[static_cast<int>(DistanceType::LONG)] = {4000.0f, 8000.0f, 320};

    // VERY_LONG (> 640 steps) - Higher speed for longer distances
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_LONG)] = {8000.0f, 16000.0f, 640};
}

// Destructor
PositionController::~PositionController()
{
    stop();
    disable();
    _instances[_motorId] = nullptr;
}

// Initialization
bool PositionController::begin()
{
    if (_initialized)
        return true;

    // Configure stepper
    _stepper.setEnablePin(_enPin);
    _stepper.setPinsInverted(false, false, true);  // Invert enable pin
    _stepper.disableOutputs();                     // Start disabled

    // Initialize and select this motor in the direction multiplexer
    if (!_dirMultiplexer.begin())
    {
        log_e("Failed to initialize direction multiplexer for motor %d", _motorId + 1);
        return false;
    }

    if (!_dirMultiplexer.selectMotor(_motorId))
    {
        log_e("Failed to select motor %d in direction multiplexer", _motorId + 1);
        return false;
    }

    // Set default speed and acceleration
    _stepper.setMaxSpeed(_speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)].maxSpeed);
    _stepper.setAcceleration(_speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)].acceleration);

    // Set current position to 0
    setCurrentPosition(0);

    _initialized = true;
    log_d("Motor %d initialized with direction multiplexer", _motorId + 1);
    return true;
}

void PositionController::setCurrentPosition(int32_t positionSteps)
{
    _stepper.setCurrentPosition(static_cast<long>(positionSteps));
    _status.currentSteps = positionSteps;
}

// Enable/Disable
void PositionController::enable()
{
    if (!_initialized)
        return;

    _stepper.enableOutputs();
    _enabled          = true;
    _status.isEnabled = true;
}

void PositionController::disable()
{
    if (!_initialized)
        return;

    stop();
    _stepper.disableOutputs();
    _enabled          = false;
    _status.isEnabled = false;
    log_i("Motor %d disabled", _motorId + 1);
}

bool PositionController::isEnabled() const
{
    return _enabled && _status.isEnabled;
}

// Position control methods
bool PositionController::moveToSteps(int32_t targetSteps, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    // Calculate movement distance in steps
    int32_t currentSteps          = getCurrentSteps();
    int32_t delta                 = targetSteps - currentSteps;
    int32_t movementDistanceSteps = abs(delta);

    // Validate movement distance
    if (!isValidMovementDistanceSteps(movementDistanceSteps))
    {
        log_w("Motor %d: Movement distance %d steps is negligible (< 6 steps), "
              "ignoring",
              _motorId + 1,
              movementDistanceSteps);
        return false;
    }

    // Calculate distance type for speed profile selection
    DistanceType distanceType = calculateDistanceTypeSteps(movementDistanceSteps);

    // Create movement command
    MovementCommand command;
    command.motorId               = _motorId;
    command.targetSteps           = targetSteps;
    command.movementType          = movementType;
    command.distanceType          = distanceType;
    command.relative              = false;
    command.priority              = 1;
    command.controlMode           = controlMode;
    command.movementDistanceSteps = movementDistanceSteps;

    // Queue the command
    return queueMovementCommand(command);
}

bool PositionController::moveRelativeSteps(int32_t deltaSteps, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    // Validate movement distance
    int32_t movementDistanceSteps = abs(deltaSteps);
    if (!isValidMovementDistanceSteps(movementDistanceSteps))
    {
        log_w("Motor %d: Movement distance %d steps is negligible (< 6 steps), "
              "ignoring",
              _motorId + 1,
              movementDistanceSteps);
        return false;
    }

    int32_t currentSteps = getCurrentSteps();
    int32_t targetSteps  = currentSteps + deltaSteps;

    // Calculate distance type for speed profile selection
    DistanceType distanceType = calculateDistanceTypeSteps(movementDistanceSteps);

    // Create movement command
    MovementCommand command;
    command.motorId               = _motorId;
    command.targetSteps           = targetSteps;
    command.movementType          = movementType;
    command.distanceType          = distanceType;
    command.relative              = true;
    command.priority              = 1;
    command.controlMode           = controlMode;
    command.movementDistanceSteps = movementDistanceSteps;

    // Queue the command
    return queueMovementCommand(command);
}

void PositionController::stop()
{
    if (!_initialized)
        return;

    _stepper.stop();
    _stepper.setCurrentPosition(_stepper.currentPosition());  // Update current position

    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        _status.isMoving          = false;
        _status.totalMovementTime = millis() - _status.movementStartTime;
        xSemaphoreGive(_statusMutex);
    }

    log_i("Motor %d stopped", _motorId + 1);
}

bool PositionController::isMoving() const
{
    return _status.isMoving && _stepper.distanceToGo() != 0;
}

bool PositionController::isAtTarget() const
{
    if (!_enabled || !_initialized)
        return false;

    int32_t currentPos = static_cast<int32_t>(_stepper.currentPosition());
    int32_t targetPos  = static_cast<int32_t>(_stepper.targetPosition());

    // Check if within accuracy threshold
    return abs(static_cast<int32_t>(currentPos - targetPos)) <= POSITION_ACCURACY_STEPS;
}

int32_t PositionController::getCurrentSteps() const
{
    if (!_initialized)
        return 0;

    return static_cast<int32_t>(const_cast<AccelStepper&>(_stepper).currentPosition());
}

int32_t PositionController::getTargetSteps() const
{
    if (!_initialized)
        return 0;

    return static_cast<int32_t>(const_cast<AccelStepper&>(_stepper).currentPosition());
    // return _status.targetSteps;
}

MotorStatus PositionController::getMotorStatus()
{
    MotorStatus status = _status;

    // Update with current values
    status.currentSteps = getCurrentSteps();
    status.targetSteps  = getTargetSteps();

    setCurrentPosition(status.currentSteps);
    status.isMoving  = isMoving();
    status.isEnabled = isEnabled();

    return status;
}

// Configuration
void PositionController::setMaxSpeed(float speedStepsPerSec)
{
    if (!_initialized)
        return;

    _stepper.setMaxSpeed(speedStepsPerSec);
}

void PositionController::setAcceleration(float accelerationStepsPerSec2)
{
    if (!_initialized)
        return;

    _stepper.setAcceleration(accelerationStepsPerSec2);
}

void PositionController::setSpeedProfile(MovementType type, float maxSpeed, float acceleration)
{
    int index = static_cast<int>(type);
    if (index >= 0 && index < 3)
    {
        _speedProfiles[index].maxSpeed     = maxSpeed;
        _speedProfiles[index].acceleration = acceleration;
    }
}

// RTOS task management
void PositionController::startPositionControlTask()
{
    if (_positionControlTaskHandle != nullptr)
        return;

    // Create queue for movement commands
    _movementCommandQueue = xQueueCreate(10, sizeof(MovementCommand));
    if (_movementCommandQueue == nullptr)
    {
        log_e("Failed to create command queue");
        return;
    }

    // Create mutex for status protection
    _statusMutex = xSemaphoreCreateMutex();
    if (_statusMutex == nullptr)
    {
        log_e("Failed to create status mutex");
        return;
    }

    // Create position control task
    BaseType_t result = xTaskCreatePinnedToCore(positionControlTask,
                                                "PositionControl",
                                                4096,
                                                nullptr,
                                                3,  // Priority
                                                &_positionControlTaskHandle,
                                                1  // Core 1
    );

    if (result == pdPASS)
    {
        log_d("Position control task started");
        esp_task_wdt_add(_positionControlTaskHandle);
    }
    else
    {
        log_e("Failed to create position control task");
    }
}

void PositionController::stopPositionControlTask()
{
    if (_positionControlTaskHandle != nullptr)
    {
        vTaskDelete(_positionControlTaskHandle);
        _positionControlTaskHandle = nullptr;
    }

    if (_movementCommandQueue != nullptr)
    {
        vQueueDelete(_movementCommandQueue);
        _movementCommandQueue = nullptr;
    }

    if (_statusMutex != nullptr)
    {
        vSemaphoreDelete(_statusMutex);
        _statusMutex = nullptr;
    }

    log_d("Position control task stopped");
}

bool PositionController::queueMovementCommand(const MovementCommand& command)
{
    if (_movementCommandQueue == nullptr)
        return false;

    return xQueueSend(_movementCommandQueue, &command, pdMS_TO_TICKS(100)) == pdTRUE;
}

// Private methods
void PositionController::updateStatus()
{
    if (!_initialized)
        return;

    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        _status.currentSteps = static_cast<int32_t>(_stepper.currentPosition());
        _status.isMoving     = _stepper.distanceToGo() != 0;
        xSemaphoreGive(_statusMutex);
    }
}

void PositionController::configureSpeedProfile(MovementType type)
{
    int index = static_cast<int>(type);
    if (index >= 0 && index < 3)
    {
        _stepper.setMaxSpeed(_speedProfiles[index].maxSpeed);
        _stepper.setAcceleration(_speedProfiles[index].acceleration);
    }
}

void PositionController::setSpeedForMovement(MovementType type)
{
    configureSpeedProfile(type);
    _status.lastMovementType = type;
}

void PositionController::setDirection(bool direction)
{
    if (!_initialized)
        return;

    // Select this motor in the multiplexer and set direction
    _dirMultiplexer.setMotorDirection(_motorId, direction);
}

bool PositionController::executeMovement(const MovementCommand& command)
{
    if (!_enabled || !_initialized)
        return false;

    // Set control mode based on command
    setControlMode(command.controlMode);

    // Calculate target steps
    int32_t targetSteps = command.targetSteps;

    // Update status
    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        _status.targetSteps       = targetSteps;
        _status.isMoving          = true;
        _status.movementStartTime = millis();
        _status.lastMovementType  = command.movementType;
        _status.controlMode       = command.controlMode;
        xSemaphoreGive(_statusMutex);
    }

    // Configure speed based on movement distance for precise control
    configureSpeedForDistanceSteps(command.movementDistanceSteps);

    // Execute movement
    _stepper.moveTo(static_cast<long>(targetSteps));

    const char* modeStr     = (command.controlMode == ControlMode::OPEN_LOOP) ? "OPEN-LOOP" : "HYBRID";
    const char* distanceStr = "";
    switch (command.distanceType)
    {
        case DistanceType::VERY_SHORT:
            distanceStr = "VERY_SHORT";
            break;
        case DistanceType::SHORT:
            distanceStr = "SHORT";
            break;
        case DistanceType::MEDIUM:
            distanceStr = "MEDIUM";
            break;
        case DistanceType::LONG:
            distanceStr = "LONG";
            break;
        case DistanceType::VERY_LONG:
            distanceStr = "VERY_LONG";
            break;
        default:
            distanceStr = "UNKNOWN";
            break;
    }

    log_d("Motor %d moving to %d steps (distance: %d steps, type: %s, mode: %s)", _motorId + 1, targetSteps, command.movementDistanceSteps, distanceStr, modeStr);

    return true;
}

float PositionController::calculateOptimalSpeedSteps(int32_t distanceSteps, MovementType type)
{
    // Calculate optimal speed based on distance and movement type
    float baseSpeed = _speedProfiles[static_cast<int>(type)].maxSpeed;

    // Adjust speed based on distance
    if (distanceSteps < 640)  // Short movements (< 10°)
        return baseSpeed * 0.5f;
    else if (distanceSteps < 5760)  // Medium movements (< 90°)
        return baseSpeed * 0.8f;
    else  // Long movements
        return baseSpeed;
}

// RTOS task function
void PositionController::positionControlTask(void* parameter)
{
    MovementCommand  command;
    TickType_t       lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency    = pdMS_TO_TICKS(10);  // 10ms task period

    log_d("Position control task running");

    while (true)
    {
        // Check for movement commands
        if (xQueueReceive(_movementCommandQueue, &command, 0) == pdTRUE)
        {
            if (command.motorId < 4 && _instances[command.motorId] != nullptr)
            {
                _instances[command.motorId]->executeMovement(command);
            }
        }

        // Update all motor controllers
        for (int i = 0; i < 4; i++)
        {
            if (_instances[i] != nullptr)
            {
                _instances[i]->runPositionControl();
            }
        }

        // Reset watchdog
        esp_task_wdt_reset();

        // Wait for next cycle
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

void PositionController::runPositionControl()
{
    if (!_enabled || !_initialized)
        return;

    // Handle direction control through multiplexer
    // Since AccelStepper is initialized with dirPin=0, we need to manually
    // control direction
    if (_stepper.distanceToGo() != 0)
    {
        // Determine direction based on whether we need to move forward or backward
        bool direction = (_stepper.distanceToGo() > 0);
        setDirection(direction);
    }

    // Run the stepper motor
    _stepper.run();

    // Apply control mode corrections if enabled
    if (_status.isMoving && isHybridModeEnabled())
    {
        applyHybridModeCorrection();
    }

    // Update status periodically
    static int32_t lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 100)  // Update every 100ms
    {
        updateStatus();
        lastStatusUpdate = millis();
    }

    // Check if movement is complete
    if (_status.isMoving && _stepper.distanceToGo() == 0)
    {
        if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            _status.isMoving          = false;
            _status.totalMovementTime = millis() - _status.movementStartTime;
            xSemaphoreGive(_statusMutex);
        }

        const char* modeStr   = (_controlMode == ControlMode::OPEN_LOOP) ? "OPEN-LOOP" : "HYBRID";
        _movementCompleteFlag = true;
        log_i("Motor %d reached target (%s). 🎯", _motorId + 1, modeStr);
    }
}

// Control mode methods
ControlMode PositionController::getControlMode() const
{
    return _controlMode;
}

bool PositionController::isHybridModeEnabled() const
{
    return _controlMode == ControlMode::HYBRID && _encoderMae3 != nullptr && (_encoderMae3->enable() == mae3::Status::Ok);
}

EncoderState PositionController::getEncoderState() const
{
    if (_encoderMae3 != nullptr)
    {
        if (_encoderMae3->enable() == mae3::Status::Ok)
        {
            std::double_t pos{0U};
            std::double_t ton{0U};
            std::double_t toff{0U};
            if (_encoderMae3->tryGetPosition(pos, ton, toff))
            {
                return {ton, toff, 0, pos, 0, Direction::UNKNOWN};  // amirr
            }
        }
    }
    return {0, 0, 0, 0, 0, Direction::UNKNOWN};
}

int32_t PositionController::getEncoderSteps()
{
    uint32_t pulse = getEncoderState().position_pulse;
    return UnitConverter::convertFromPulses(pulse).TO_STEPS;
}

// Global functions for RTOS integration
void initializePositionControllers()
{
    // Initialize RTOS components
    PositionController::startPositionControlTask();
}

void startPositionControlSystem()
{
    // Enable all position controllers
    for (int i = 0; i < 4; i++)
    {
        if (PositionController::_instances[i] != nullptr)
        {
            PositionController::_instances[i]->enable();
        }
    }
}

void stopPositionControlSystem()
{
    // Stop all position controllers
    for (int i = 0; i < 4; i++)
    {
        if (PositionController::_instances[i] != nullptr)
        {
            PositionController::_instances[i]->stop();
            PositionController::_instances[i]->disable();
        }
    }

    // Stop RTOS task
    PositionController::stopPositionControlTask();
}

void PositionController::applyHybridModeCorrection()
{
    if (!isHybridModeEnabled() || !_status.isMoving)
        return;

    // In hybrid mode, we only read the encoder once at the start
    // No continuous correction is applied during movement
    // The encoder reading was already done in setControlMode()
}

// Control mode methods
void PositionController::setControlMode(ControlMode mode)
{
    _controlMode = mode;

    switch (mode)
    {
        case ControlMode::OPEN_LOOP:
            // No special setup needed for open-loop
            _status.controlMode = ControlMode::OPEN_LOOP;
            break;

        case ControlMode::HYBRID:
            // Enable hybrid mode - read encoder once at start
            if (_encoderMae3 != nullptr && (_encoderMae3->enable() == mae3::Status::Ok))
            {
                _status.controlMode = ControlMode::HYBRID;
                // Read current encoder position and use it as starting position
                int32_t encoderSteps = getEncoderSteps();
                _status.encoderSteps = encoderSteps;
                log_d("Motor %d: Hybrid mode - starting position from encoder: %d steps", _motorId + 1, encoderSteps);
            }
            else
            {
                log_w("Motor %d: Cannot enable hybrid mode - encoder not available", _motorId + 1);
                _controlMode        = ControlMode::OPEN_LOOP;
                _status.controlMode = ControlMode::OPEN_LOOP;
            }
            break;
    }
}

// Distance-based control methods
DistanceType PositionController::calculateDistanceTypeSteps(int32_t distanceSteps)
{
    if (distanceSteps < 6)
        return DistanceType::NEGLIGIBLE;
    else if (distanceSteps < 32)
        return DistanceType::VERY_SHORT;
    else if (distanceSteps < 64)
        return DistanceType::SHORT;
    else if (distanceSteps < 320)
        return DistanceType::MEDIUM;
    else if (distanceSteps < 640)
        return DistanceType::LONG;
    else
        return DistanceType::VERY_LONG;
}

bool PositionController::isValidMovementDistanceSteps(int32_t distanceSteps)
{
    return distanceSteps >= 6;  // Ignore movements smaller than 6 steps
}

void PositionController::setDistanceBasedSpeedProfile(DistanceType distanceType)
{
    if (distanceType == DistanceType::NEGLIGIBLE)
        return;

    int index = static_cast<int>(distanceType);
    if (index >= 0 && index < 6)
    {
        _stepper.setMaxSpeed(_distanceSpeedProfiles[index].maxSpeed);
        _stepper.setAcceleration(_distanceSpeedProfiles[index].acceleration);
    }
}

float PositionController::calculateOptimalSpeedForDistanceSteps(int32_t distanceSteps)
{
    DistanceType distanceType = calculateDistanceTypeSteps(distanceSteps);
    if (distanceType == DistanceType::NEGLIGIBLE)
        return 0.0f;

    int index = static_cast<int>(distanceType);
    if (index >= 0 && index < 6)
    {
        float baseSpeed = _distanceSpeedProfiles[index].maxSpeed;

        // Fine-tune speed based on exact distance within the range
        float speedMultiplier = 1.0f;

        switch (distanceType)
        {
            case DistanceType::VERY_SHORT:
                // 6 - 32 steps: Linear interpolation
                speedMultiplier = 0.5f + (distanceSteps - 6) * 0.5f / 26.0f;
                break;
            case DistanceType::SHORT:
                // 32 - 64 steps: Linear interpolation
                speedMultiplier = 0.7f + (distanceSteps - 32) * 0.3f / 32.0f;
                break;
            case DistanceType::MEDIUM:
                // 64 - 320 steps: Linear interpolation
                speedMultiplier = 0.8f + (distanceSteps - 64) * 0.2f / 256.0f;
                break;
            case DistanceType::LONG:
                // 320 - 640 steps: Linear interpolation
                speedMultiplier = 0.9f + (distanceSteps - 320) * 0.1f / 320.0f;
                break;
            case DistanceType::VERY_LONG:
                // > 640 steps: Full speed for longer distances
                speedMultiplier = 1.0f;
                break;
            default:
                speedMultiplier = 1.0f;
                break;
        }

        return baseSpeed * speedMultiplier;
    }

    return _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)].maxSpeed;
}

float PositionController::calculateOptimalAccelerationForDistanceSteps(int32_t distanceSteps)
{
    DistanceType distanceType = calculateDistanceTypeSteps(distanceSteps);
    if (distanceType == DistanceType::NEGLIGIBLE)
        return 0.0f;

    int index = static_cast<int>(distanceType);
    if (index >= 0 && index < 6)
    {
        float baseAcceleration = _distanceSpeedProfiles[index].acceleration;

        // Fine-tune acceleration based on exact distance within the range
        float accelerationMultiplier = 1.0f;

        switch (distanceType)
        {
            case DistanceType::VERY_SHORT:
                // 6 - 32 steps: Lower acceleration for precision
                accelerationMultiplier = 0.3f + (distanceSteps - 6) * 0.4f / 26.0f;
                break;
            case DistanceType::SHORT:
                // 32 - 64 steps: Moderate acceleration
                accelerationMultiplier = 0.5f + (distanceSteps - 32) * 0.3f / 32.0f;
                break;
            case DistanceType::MEDIUM:
                // 64 - 320 steps: Balanced acceleration
                accelerationMultiplier = 0.7f + (distanceSteps - 64) * 0.2f / 256.0f;
                break;
            case DistanceType::LONG:
                // 320 - 640 steps: Higher acceleration
                accelerationMultiplier = 0.8f + (distanceSteps - 320) * 0.2f / 320.0f;
                break;
            case DistanceType::VERY_LONG:
                // > 640 steps: Full acceleration for longer distances
                accelerationMultiplier = 1.0f;
                break;
            default:
                accelerationMultiplier = 1.0f;
                break;
        }

        return baseAcceleration * accelerationMultiplier;
    }

    return _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)].acceleration;
}

void PositionController::configureSpeedForDistanceSteps(int32_t distanceSteps)
{
    if (!isValidMovementDistanceSteps(distanceSteps))
        return;

    float optimalSpeed        = calculateOptimalSpeedForDistanceSteps(distanceSteps);
    float optimalAcceleration = calculateOptimalAccelerationForDistanceSteps(distanceSteps);

    _stepper.setMaxSpeed(optimalSpeed);
    _stepper.setAcceleration(optimalAcceleration);

    log_d("Motor %d: Distance %d steps - Speed: %.0f steps/s, Accel: %.0f steps/s²", _motorId + 1, distanceSteps, optimalSpeed, optimalAcceleration);
}

void PositionController::attachOnComplete(void (*callback)())
{
    _onComplete = callback;
}
void PositionController::handleMovementComplete()
{
    if (_movementCompleteFlag)
    {
        _movementCompleteFlag = false;
        if (_onComplete)
            _onComplete();
    }
}
void PositionController::setMovementCompleteFlag(bool flag)
{
    _movementCompleteFlag = flag;
}

MotorType PositionController::getMotorType() const
{
    if (_motorId == 0)
        return MotorType::LINEAR;

    return MotorType::ROTATIONAL;
}
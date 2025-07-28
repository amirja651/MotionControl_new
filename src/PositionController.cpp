#include "PositionController.h"
#include <cmath>
#include <esp_task_wdt.h>

// Static member initialization
TaskHandle_t        PositionController::_positionControlTaskHandle = nullptr;
QueueHandle_t       PositionController::_movementCommandQueue      = nullptr;
SemaphoreHandle_t   PositionController::_statusMutex               = nullptr;
PositionController* PositionController::_instances[4]              = {nullptr};

// Constructor with encoder
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer,
                                       uint16_t stepPin, uint16_t enPin, MAE3Encoder& encoder)
    : _driver(driver),
      _stepper(AccelStepper::DRIVER, stepPin, 0),  // dirPin will be handled by multiplexer
      _encoder(&encoder),
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
    _status.currentAngle      = 0.0f;
    _status.targetAngle       = 0.0f;
    _status.isMoving          = false;
    _status.isEnabled         = false;
    _status.currentSteps      = 0;
    _status.targetSteps       = 0;
    _status.lastMovementType  = MovementType::MEDIUM_RANGE;
    _status.movementStartTime = 0;
    _status.totalMovementTime = 0;
    _status.controlMode       = ControlMode::OPEN_LOOP;
    _status.positionError     = 0.0f;
    _status.encoderAngle      = 0.0f;

    // Store instance for RTOS access
    _instances[motorId] = this;

    // Initialize speed profiles with default values
    _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {2000.0f, 4000.0f};   // Increased from 1000
    _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {4000.0f, 8000.0f};   // Increased from 2000
    _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {8000.0f, 16000.0f};  // Increased from 4000

    // Initialize distance-based speed profiles for precise control
    // NEGLIGIBLE (< 0.01°) - Not used, but defined for completeness
    _distanceSpeedProfiles[static_cast<int>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f, 0.0f};

    // VERY_SHORT (0.01° - 0.5°) - Very slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f, 0.05f};

    // SHORT (0.5° - 1°) - Slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::SHORT)] = {1000.0f, 2000.0f, 0.1f};

    // MEDIUM (1° - 5°) - Balanced speed and precision
    _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)] = {2000.0f, 4000.0f, 0.2f};

    // LONG (5° - 10°) - Moderate speed
    _distanceSpeedProfiles[static_cast<int>(DistanceType::LONG)] = {4000.0f, 8000.0f, 0.5f};

    // VERY_LONG (> 10°) - Higher speed for longer distances
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_LONG)] = {8000.0f, 16000.0f, 1.0f};
}

// Constructor without encoder (for demo)
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer,
                                       uint16_t stepPin, uint16_t enPin)
    : _driver(driver),
      _stepper(AccelStepper::DRIVER, stepPin, 0),  // dirPin will be handled by multiplexer
      _encoder(nullptr),
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
    _status.currentAngle      = 0.0f;
    _status.targetAngle       = 0.0f;
    _status.isMoving          = false;
    _status.isEnabled         = false;
    _status.currentSteps      = 0;
    _status.targetSteps       = 0;
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
    // NEGLIGIBLE (< 0.01°) - Not used, but defined for completeness
    _distanceSpeedProfiles[static_cast<int>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f, 0.0f};

    // VERY_SHORT (0.01° - 0.5°) - Very slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f, 0.05f};

    // SHORT (0.5° - 1°) - Slow and precise
    _distanceSpeedProfiles[static_cast<int>(DistanceType::SHORT)] = {1000.0f, 2000.0f, 0.1f};

    // MEDIUM (1° - 5°) - Balanced speed and precision
    _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)] = {2000.0f, 4000.0f, 0.2f};

    // LONG (5° - 10°) - Moderate speed
    _distanceSpeedProfiles[static_cast<int>(DistanceType::LONG)] = {4000.0f, 8000.0f, 0.5f};

    // VERY_LONG (> 10°) - Higher speed for longer distances
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_LONG)] = {8000.0f, 16000.0f, 1.0f};
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
    log_i("Motor %d initialized with direction multiplexer", _motorId + 1);
    return true;
}

void PositionController::setCurrentPosition(int32_t position)
{
    _stepper.setCurrentPosition(position);
    _status.currentSteps = position;
    _status.currentAngle = convertFromMSteps(position).DEGREES_FROM_STEPS;
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
bool PositionController::moveToAngle(float targetAngle, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    // Wrap target angle to 0-360 range
    targetAngle = wrapAngle(targetAngle);

    // Calculate current angle and movement distance
    float currentAngle     = wrapAngle(getCurrentAngle());
    float delta            = calculateShortestPath(currentAngle, targetAngle);
    float movementDistance = abs(delta);

    // Validate movement distance
    if (!isValidMovementDistance(movementDistance))
    {
        log_w("Motor %d: Movement distance %.3f° is negligible (< 0.01°), ignoring", _motorId + 1, movementDistance);
        return false;
    }

    // Calculate distance type for speed profile selection
    DistanceType distanceType = calculateDistanceType(movementDistance);

    // Create movement command
    MovementCommand command;
    command.motorId          = _motorId;
    command.targetAngle      = targetAngle;
    command.movementType     = movementType;
    command.distanceType     = distanceType;
    command.relative         = false;
    command.priority         = 1;
    command.controlMode      = controlMode;
    command.movementDistance = movementDistance;

    // Queue the command
    return queueMovementCommand(command);
}

bool PositionController::moveRelative(float deltaAngle, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    // Validate movement distance
    float movementDistance = abs(deltaAngle);
    if (!isValidMovementDistance(movementDistance))
    {
        log_w("Motor %d: Movement distance %.3f° is negligible (< 0.01°), ignoring", _motorId + 1, movementDistance);
        return false;
    }

    float currentAngle = getCurrentAngle();
    float targetAngle  = wrapAngle(currentAngle + deltaAngle);

    // Calculate distance type for speed profile selection
    DistanceType distanceType = calculateDistanceType(movementDistance);

    // Create movement command
    MovementCommand command;
    command.motorId          = _motorId;
    command.targetAngle      = targetAngle;
    command.movementType     = movementType;
    command.distanceType     = distanceType;
    command.relative         = true;
    command.priority         = 1;
    command.controlMode      = controlMode;
    command.movementDistance = movementDistance;

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
    ConvertValuesFromDegrees cvfd = convertFromDegrees(POSITION_ACCURACY_DEGREES);
    return abs(static_cast<int32_t>(currentPos - targetPos)) <= cvfd.STEPS_FROM_DEGREES;
}

// Status and information
float PositionController::getCurrentAngle() const
{
    if (!_initialized)
        return 0.0f;

    int32_t currentSteps = static_cast<int32_t>(const_cast<AccelStepper&>(_stepper).currentPosition());
    return convertFromMSteps(currentSteps).DEGREES_FROM_STEPS;
}

float PositionController::getTargetAngle() const
{
    return _status.targetAngle;
}

int32_t PositionController::getCurrentSteps() const
{
    if (!_initialized)
        return 0;

    return static_cast<int32_t>(const_cast<AccelStepper&>(_stepper).currentPosition());
}

int32_t PositionController::getTargetSteps() const
{
    return _status.targetSteps;
}

MotorStatus PositionController::getStatus()
{
    MotorStatus status = _status;

    // Update with current values
    status.targetAngle  = wrapAngle(getTargetAngle());
    status.currentAngle = wrapAngle(getCurrentAngle());
    int32_t steps       = convertFromDegrees(status.currentAngle).STEPS_FROM_DEGREES;
    setCurrentPosition(steps);
    // status.currentSteps = getCurrentSteps();
    status.isMoving  = isMoving();
    status.isEnabled = isEnabled();

    // Update closed-loop information
    if (isClosedLoopEnabled())
    {
        status.positionError = calculatePositionError();
    }

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

// Angle utilities
float PositionController::wrapAngle(float angle)
{
    if (0)
    {
        angle = fmod(angle, 360.0f);
        if (angle < 0.0f)
            angle += 360.0f;
    }
    return angle;
}

float PositionController::calculateShortestPath(float currentAngle, float targetAngle)
{
    float delta = targetAngle - currentAngle;
    if (0)
    {
        delta = fmod(delta + 540.0f, 360.0f) - 180.0f;  // نگاشت به [-180, +180]
    }
    return delta;
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
    BaseType_t result = xTaskCreatePinnedToCore(positionControlTask, "PositionControl", 4096, nullptr,
                                                3,  // Priority
                                                &_positionControlTaskHandle,
                                                1  // Core 1
    );

    if (result == pdPASS)
    {
        log_i("Position control task started");
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

    log_i("Position control task stopped");
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
        _status.currentAngle = convertFromMSteps(_status.currentSteps).DEGREES_FROM_STEPS;
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

bool PositionController::executeMovement(const MovementCommand& command)  // amir
{
    if (!_enabled || !_initialized)
        return false;

    // Set control mode based on command
    setControlMode(command.controlMode);

    // Calculate target steps
    int32_t targetSteps;
    if (command.relative)
    {
        targetSteps = _status.currentSteps + convertFromDegrees(command.targetAngle).STEPS_FROM_DEGREES;
    }
    else
    {
        targetSteps = convertFromDegrees(command.targetAngle).STEPS_FROM_DEGREES;
    }

    // Update status
    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        _status.targetAngle       = command.targetAngle;
        _status.targetSteps       = targetSteps;
        _status.isMoving          = true;
        _status.movementStartTime = millis();
        _status.lastMovementType  = command.movementType;
        _status.controlMode       = command.controlMode;
        xSemaphoreGive(_statusMutex);
    }

    // Configure speed based on movement distance for precise control
    configureSpeedForDistance(command.movementDistance);

    // Execute movement
    _stepper.moveTo(static_cast<long>(targetSteps));

    const char* modeStr     = (command.controlMode == ControlMode::OPEN_LOOP)     ? "OPEN-LOOP"
                              : (command.controlMode == ControlMode::CLOSED_LOOP) ? "CLOSED-LOOP"
                                                                                  : "HYBRID";
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

    log_i("Motor %d moving to %.2f degrees (distance: %.3f°, type: %s, mode: %s)", _motorId + 1, command.targetAngle,
          command.movementDistance, distanceStr, modeStr);

    return true;
}

float PositionController::calculateOptimalSpeed(float distance, MovementType type)
{
    // Calculate optimal speed based on distance and movement type
    float baseSpeed = _speedProfiles[static_cast<int>(type)].maxSpeed;

    // Adjust speed based on distance
    if (distance < 10.0f)  // Short movements
        return baseSpeed * 0.5f;
    else if (distance < 90.0f)  // Medium movements
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

    log_i("Position control task running");

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
    // Since AccelStepper is initialized with dirPin=0, we need to manually control direction
    if (_stepper.distanceToGo() != 0)
    {
        // Determine direction based on whether we need to move forward or backward
        bool direction = (_stepper.distanceToGo() > 0);
        setDirection(direction);
    }

    // Run the stepper motor
    _stepper.run();

    // Apply control mode corrections if enabled
    if (_status.isMoving)
    {
        if (isClosedLoopEnabled())
        {
            applyClosedLoopCorrection();
        }
        else if (isHybridModeEnabled())
        {
            applyHybridModeCorrection();
        }
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

        const char* modeStr = (_controlMode == ControlMode::OPEN_LOOP)     ? "OPEN-LOOP"
                              : (_controlMode == ControlMode::CLOSED_LOOP) ? "CLOSED-LOOP"
                                                                           : "HYBRID";

        if (_controlMode == ControlMode::CLOSED_LOOP)
        {
            float finalError = calculatePositionError();
            log_i("Motor %d reached target (%s). Final error: %.2f°", _motorId + 1, modeStr, finalError);
        }
        else
        {
            _movementCompleteFlag = true;
            log_i("Motor %d reached target (%s).", _motorId + 1, modeStr);
        }
    }
}

// Control mode methods
ControlMode PositionController::getControlMode() const
{
    return _controlMode;
}

bool PositionController::isClosedLoopEnabled() const
{
    return _controlMode == ControlMode::CLOSED_LOOP && _encoder != nullptr && _encoder->isEnabled();
}

bool PositionController::isHybridModeEnabled() const
{
    return _controlMode == ControlMode::HYBRID && _encoder != nullptr && _encoder->isEnabled();
}

float PositionController::getEncoderAngle()
{
    if (_encoder != nullptr && _encoder->isEnabled())
    {
        _encoder->processPWM();
        EncoderState encoderState = _encoder->getState();
        return wrapAngle(encoderState.position_degrees);
    }
    return 0.0f;
}

float PositionController::calculatePositionError()
{
    if (!isClosedLoopEnabled())
        return 0.0f;

    float targetAngle  = _status.targetAngle;
    float encoderAngle = getEncoderAngle();

    // Calculate shortest path error
    float error = calculateShortestPath(encoderAngle, targetAngle);

    _status.encoderAngle  = encoderAngle;
    _status.positionError = error;

    return error;
}

void PositionController::applyClosedLoopCorrection()
{
    if (!isClosedLoopEnabled() || !_status.isMoving)
        return;

    float error = calculatePositionError();

    // Only apply correction if error is significant (more than 0.1 degrees)
    if (abs(error) > 0.1f)
    {
        // Simple proportional correction
        float   correctionSteps = convertFromDegrees(error).STEPS_FROM_DEGREES;
        int32_t currentPos      = _stepper.currentPosition();
        int32_t newTarget       = currentPos + static_cast<int32_t>(correctionSteps);

        // Apply correction
        _stepper.moveTo(newTarget);
    }
}

ConvertValuesFromDegrees PositionController::convertFromDegrees(float degrees, int32_t microsteps,
                                                                int32_t resolution) const
{
    ConvertValuesFromDegrees convert;
    convert.PULSES_FROM_DEGREES = static_cast<int32_t>(std::round(degrees * (resolution / 360.0f)));
    convert.STEPS_FROM_DEGREES  = static_cast<int32_t>(std::round(degrees * (microsteps / 360.0f)));
    return convert;
}

ConvertValuesFromPulses PositionController::convertFromPulses(int32_t pulses, int32_t microsteps,
                                                              int32_t resolution) const
{
    ConvertValuesFromPulses convert;
    convert.DEGREES_FROM_PULSES = static_cast<float>(pulses * (360.0f / resolution));
    convert.STEPS_FROM_PULSES   = static_cast<int32_t>(std::round(pulses * (microsteps / resolution)));
    return convert;
}

ConvertValuesFromSteps PositionController::convertFromMSteps(int32_t steps, int32_t microsteps,
                                                             int32_t resolution) const
{
    ConvertValuesFromSteps convert;
    convert.PULSES_FROM_STEPS  = static_cast<int32_t>(std::round(steps * (resolution / microsteps)));
    convert.DEGREES_FROM_STEPS = static_cast<float>(steps * (360.0f / microsteps));
    return convert;
}

float PositionController::calculateMotorAngleFromReference(float newPixel, float refPixel, float refMotorDeg)
{
    float deltaPixel     = newPixel - refPixel;
    float delta_mm       = deltaPixel * PIXEL_SIZE_MM;
    float mirrorAngleRad = atan(delta_mm / CAMERA_TO_MIRROR_LENGTH_MM);
    float motorAngleDeg  = refMotorDeg + (mirrorAngleRad * 180.0 / M_PI);

    return motorAngleDeg;
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
            break;

        case ControlMode::CLOSED_LOOP:
            // Enable continuous closed-loop control
            if (_encoder != nullptr && _encoder->isEnabled())
            {
                _status.controlMode = ControlMode::CLOSED_LOOP;
            }
            else
            {
                log_w("Motor %d: Cannot enable closed-loop - encoder not available", _motorId + 1);
                _controlMode        = ControlMode::OPEN_LOOP;
                _status.controlMode = ControlMode::OPEN_LOOP;
            }
            break;

        case ControlMode::HYBRID:
            // Enable hybrid mode - read encoder once at start
            if (_encoder != nullptr && _encoder->isEnabled())
            {
                _status.controlMode = ControlMode::HYBRID;
                // Read current encoder position and use it as starting position
                float encoderAngle   = getEncoderAngle();
                _status.encoderAngle = encoderAngle;
                log_i("Motor %d: Hybrid mode - starting position from encoder: %.2f degrees", _motorId + 1,
                      encoderAngle);
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
DistanceType PositionController::calculateDistanceType(float distance)
{
    if (distance < 0.05f)
        return DistanceType::NEGLIGIBLE;
    else if (distance < 0.5f)
        return DistanceType::VERY_SHORT;
    else if (distance < 1.0f)
        return DistanceType::SHORT;
    else if (distance < 5.0f)
        return DistanceType::MEDIUM;
    else if (distance < 10.0f)
        return DistanceType::LONG;
    else
        return DistanceType::VERY_LONG;
}

bool PositionController::isValidMovementDistance(float distance)
{
    return distance >= 0.01f;  // Ignore movements smaller than 0.01°
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

float PositionController::calculateOptimalSpeedForDistance(float distance)
{
    DistanceType distanceType = calculateDistanceType(distance);
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
                // 0.01° - 0.5°: Linear interpolation
                speedMultiplier = 0.5f + (distance - 0.01f) * 0.5f / 0.4f;
                break;
            case DistanceType::SHORT:
                // 0.5° - 1°: Linear interpolation
                speedMultiplier = 0.7f + (distance - 0.5f) * 0.3f / 0.5f;
                break;
            case DistanceType::MEDIUM:
                // 1° - 5°: Linear interpolation
                speedMultiplier = 0.8f + (distance - 1.0f) * 0.2f / 4.0f;
                break;
            case DistanceType::LONG:
                // 5° - 10°: Linear interpolation
                speedMultiplier = 0.9f + (distance - 5.0f) * 0.1f / 5.0f;
                break;
            case DistanceType::VERY_LONG:
                // > 10°: Full speed for longer distances
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

float PositionController::calculateOptimalAccelerationForDistance(float distance)
{
    DistanceType distanceType = calculateDistanceType(distance);
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
                // 0.01° - 0.5°: Lower acceleration for precision
                accelerationMultiplier = 0.3f + (distance - 0.01f) * 0.4f / 0.4f;
                break;
            case DistanceType::SHORT:
                // 0.5° - 1°: Moderate acceleration
                accelerationMultiplier = 0.5f + (distance - 0.5f) * 0.3f / 0.5f;
                break;
            case DistanceType::MEDIUM:
                // 1° - 5°: Balanced acceleration
                accelerationMultiplier = 0.7f + (distance - 1.0f) * 0.2f / 4.0f;
                break;
            case DistanceType::LONG:
                // 5° - 10°: Higher acceleration
                accelerationMultiplier = 0.8f + (distance - 5.0f) * 0.2f / 5.0f;
                break;
            case DistanceType::VERY_LONG:
                // > 10°: Full acceleration for longer distances
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

void PositionController::configureSpeedForDistance(float distance)
{
    if (!isValidMovementDistance(distance))
        return;

    float optimalSpeed        = calculateOptimalSpeedForDistance(distance);
    float optimalAcceleration = calculateOptimalAccelerationForDistance(distance);

    _stepper.setMaxSpeed(optimalSpeed);
    _stepper.setAcceleration(optimalAcceleration);

    log_i("Motor %d: Distance %.3f° - Speed: %.0f steps/s, Accel: %.0f steps/s²", _motorId + 1, distance, optimalSpeed,
          optimalAcceleration);
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
#include "PositionController.h"
#include <cmath>
#include <esp_task_wdt.h>

// Static member initialization
TaskHandle_t        PositionController::_positionControlTaskHandle = nullptr;
QueueHandle_t       PositionController::_movementCommandQueue      = nullptr;
SemaphoreHandle_t   PositionController::_statusMutex               = nullptr;
PositionController* PositionController::_instances[4]              = {nullptr};

// Constructor with encoder
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, uint16_t dirPin, uint16_t stepPin, uint16_t enPin, MAE3Encoder& encoder)
    : _driver(driver), _stepper(AccelStepper::DRIVER, stepPin, dirPin), _encoder(&encoder), _dirPin(dirPin), _stepPin(stepPin), _enPin(enPin), _motorId(motorId), _status{}, _enabled(false), _initialized(false), _closedLoopEnabled(false)
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
    _status.isClosedLoop      = false;
    _status.positionError     = 0.0f;
    _status.encoderAngle      = 0.0f;

    // Store instance for RTOS access
    _instances[motorId] = this;

    // Initialize speed profiles with default values
    _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {2000.0f, 4000.0f};   // Increased from 1000
    _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {4000.0f, 8000.0f};   // Increased from 2000
    _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {8000.0f, 16000.0f};  // Increased from 4000

    // _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {1000.0f, 2000.0f};  // Slow and precise
    // _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {2000.0f, 4000.0f};  // Balanced
    // _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {4000.0f, 8000.0f};  // Fast
}

// Constructor without encoder (for demo)
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, uint16_t dirPin, uint16_t stepPin, uint16_t enPin)
    : _driver(driver), _stepper(AccelStepper::DRIVER, stepPin, dirPin), _encoder(nullptr), _dirPin(dirPin), _stepPin(stepPin), _enPin(enPin), _motorId(motorId), _status{}, _enabled(false), _initialized(false), _closedLoopEnabled(false)
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

    // Set default speed and acceleration
    _stepper.setMaxSpeed(_speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)].maxSpeed);
    _stepper.setAcceleration(_speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)].acceleration);

    // Set current position to 0
    setCurrentPosition(0);

    _initialized = true;
    Serial.printf("[PositionController] Motor %d initialized\n", _motorId + 1);
    return true;
}

void PositionController::setCurrentPosition(int32_t position)
{
    Serial.printf("[PositionController] Motor %d setCurrentPosition: %d\n", _motorId + 1, position);
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
    Serial.printf("[PositionController] Motor %d disabled\n", _motorId + 1);
}

bool PositionController::isEnabled() const
{
    return _enabled && _status.isEnabled;
}

// Position control methods
bool PositionController::moveToAngle(float targetAngle, MovementType movementType, bool closedLoop)
{
    if (!_enabled || !_initialized)
        return false;
    // Serial.printf("[PositionController] Motor %d targetAngle: %.2f\n", _motorId + 1, targetAngle);

    // Wrap target angle to 0-360 range
    targetAngle = wrapAngle(targetAngle);
    // Serial.printf("[PositionController] Motor %d targetAngle: %.2f\n", _motorId + 1, targetAngle);

    // Read encoder value and use it as starting position
    float currentAngle = wrapAngle(getCurrentAngle());
    // Serial.printf("[PositionController] Motor %d currentAngle: %.2f\n", _motorId + 1, currentAngle);

    float delta = calculateShortestPath(currentAngle, targetAngle);
    // Serial.printf("[PositionController] Motor %d delta: %.2f\n", _motorId + 1, delta);

    float adjustedTarget = currentAngle + delta;
    // Serial.printf("[PositionController] Motor %d adjustedTarget: %.2f\n", _motorId + 1, adjustedTarget);

    // Create movement command
    MovementCommand command;
    command.motorId      = _motorId;
    command.targetAngle  = adjustedTarget;
    command.movementType = movementType;
    command.relative     = false;
    command.priority     = 1;
    command.closedLoop   = closedLoop;

    // Queue the command
    return queueMovementCommand(command);
}

bool PositionController::moveRelative(float deltaAngle, MovementType movementType, bool closedLoop)
{
    if (!_enabled || !_initialized)
        return false;

    float currentAngle = getCurrentAngle();
    float targetAngle  = wrapAngle(currentAngle + deltaAngle);

    // Create movement command
    MovementCommand command;
    command.motorId      = _motorId;
    command.targetAngle  = targetAngle;
    command.movementType = movementType;
    command.relative     = true;
    command.priority     = 1;
    command.closedLoop   = closedLoop;

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

    Serial.printf("[PositionController] Motor %d stopped\n", _motorId + 1);
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
        Serial.println("[PositionController] Failed to create command queue");
        return;
    }

    // Create mutex for status protection
    _statusMutex = xSemaphoreCreateMutex();
    if (_statusMutex == nullptr)
    {
        Serial.println("[PositionController] Failed to create status mutex");
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
        Serial.println("[PositionController] Position control task started");
        esp_task_wdt_add(_positionControlTaskHandle);
    }
    else
    {
        Serial.println("[PositionController] Failed to create position control task");
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

    Serial.println("[PositionController] Position control task stopped");
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

bool PositionController::executeMovement(const MovementCommand& command)
{
    if (!_enabled || !_initialized)
        return false;

    // Enable/disable closed-loop based on command
    if (command.closedLoop)
    {
        enableClosedLoop();
    }
    else
    {
        disableClosedLoop();
    }

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
        _status.isClosedLoop      = command.closedLoop;
        xSemaphoreGive(_statusMutex);
    }

    // Configure speed for movement type
    setSpeedForMovement(command.movementType);

    // Execute movement
    _stepper.moveTo(static_cast<long>(targetSteps));

    Serial.printf("[PositionController] Motor %d moving to %.2f degrees (type: %d, closed-loop: %s)\n", _motorId + 1, command.targetAngle, static_cast<int>(command.movementType), command.closedLoop ? "YES" : "NO");

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

    Serial.println("[PositionController] Position control task running");

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

    // Run the stepper motor
    _stepper.run();

    // Apply closed-loop correction if enabled
    if (isClosedLoopEnabled() && _status.isMoving)
    {
        applyClosedLoopCorrection();
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

        if (isClosedLoopEnabled())
        {
            float finalError = calculatePositionError();
            Serial.printf("[PositionController] Motor %d reached target (closed-loop). Final error: %.2f°\n", _motorId + 1, finalError);
        }
        else
        {
            Serial.printf("[PositionController] Motor %d reached target (open-loop).\n", _motorId + 1);
        }
    }
}

// Closed-loop control methods
void PositionController::enableClosedLoop()
{
    if (_encoder != nullptr && _encoder->isEnabled())
    {
        _closedLoopEnabled   = true;
        _status.isClosedLoop = true;
        Serial.printf("[PositionController] Motor %d closed-loop control enabled\n", _motorId + 1);
    }
    else
    {
        Serial.printf("[PositionController] Motor %d cannot enable closed-loop: encoder not available\n", _motorId + 1);
    }
}

void PositionController::disableClosedLoop()
{
    _closedLoopEnabled   = false;
    _status.isClosedLoop = false;
    Serial.printf("[PositionController] Motor %d closed-loop control disabled\n", _motorId + 1);
}

bool PositionController::isClosedLoopEnabled() const
{
    return _closedLoopEnabled && _encoder != nullptr && _encoder->isEnabled();
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

        // Serial.printf("[PositionController] Motor %d closed-loop correction: error=%.2f°, correction=%d steps\n", _motorId + 1, error, static_cast<int>(correctionSteps));
    }
}

ConvertValuesFromDegrees PositionController::convertFromDegrees(float degrees, int32_t microsteps, int32_t resolution) const
{
    ConvertValuesFromDegrees convert;
    convert.PULSES_FROM_DEGREES = static_cast<int32_t>(std::round(degrees * (resolution / 360.0f)));
    convert.STEPS_FROM_DEGREES  = static_cast<int32_t>(std::round(degrees * (microsteps / 360.0f)));
    return convert;
}

ConvertValuesFromPulses PositionController::convertFromPulses(int32_t pulses, int32_t microsteps, int32_t resolution) const
{
    ConvertValuesFromPulses convert;
    convert.DEGREES_FROM_PULSES = static_cast<float>(pulses * (360.0f / resolution));
    convert.STEPS_FROM_PULSES   = static_cast<int32_t>(std::round(pulses * (microsteps / resolution)));
    return convert;
}

ConvertValuesFromSteps PositionController::convertFromMSteps(int32_t steps, int32_t microsteps, int32_t resolution) const
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

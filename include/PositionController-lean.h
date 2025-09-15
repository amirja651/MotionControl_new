// =============================
// PositionController.cpp — lean RT rewrite (drop‑in)
// Notes:
// - API preserved (no header changes required)
// - Fixes: const‑correct isMoving(), correct getTargetSteps(), less logging in hot paths
// - Uses esp_timer_get_time() (µs) → monotonic; avoid millis() rollover
// - Status updates made lightweight; direction set only when moving
// - Queue/Task creation exactly once; safe deletion
// =============================

#include "PositionController.h"

#include <cmath>
#include <esp_task_wdt.h>
#include <esp_timer.h>

// ---- Small helpers
static inline uint32_t now_ms() noexcept
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Static member initialization
TaskHandle_t        PositionController::_positionControlTaskHandle = nullptr;
QueueHandle_t       PositionController::_movementCommandQueue      = nullptr;
SemaphoreHandle_t   PositionController::_statusMutex               = nullptr;
PositionController* PositionController::_instances[4]              = {nullptr};

// -----------------------------
// Constructors / Destructor
// -----------------------------
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin, mae3::Mae3Encoder& encoderMae3)
    : _driver(driver),
      _stepper(AccelStepper::DRIVER, stepPin, 0 /*dir via multiplexer*/),
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

    _instances[motorId] = this;

    _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {2000.0f, 4000.0f};
    _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {4000.0f, 8000.0f};
    _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {8000.0f, 16000.0f};

    _distanceSpeedProfiles[static_cast<int>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f, 0};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f, 6};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::SHORT)]      = {1000.0f, 2000.0f, 32};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)]     = {2000.0f, 4000.0f, 64};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::LONG)]       = {4000.0f, 8000.0f, 320};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_LONG)]  = {8000.0f, 16000.0f, 640};
}

PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin)
    : _driver(driver),
      _stepper(AccelStepper::DRIVER, stepPin, 0 /*dir via multiplexer*/),
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
    _status.motorId           = motorId;
    _status.currentSteps      = 0;
    _status.targetSteps       = 0;
    _status.isMoving          = false;
    _status.isEnabled         = false;
    _status.lastMovementType  = MovementType::MEDIUM_RANGE;
    _status.movementStartTime = 0;
    _status.totalMovementTime = 0;

    _instances[motorId] = this;

    _speedProfiles[static_cast<int>(MovementType::SHORT_RANGE)]  = {2000.0f, 4000.0f};
    _speedProfiles[static_cast<int>(MovementType::MEDIUM_RANGE)] = {4000.0f, 8000.0f};
    _speedProfiles[static_cast<int>(MovementType::LONG_RANGE)]   = {8000.0f, 16000.0f};

    _distanceSpeedProfiles[static_cast<int>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f, 0};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f, 6};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::SHORT)]      = {1000.0f, 2000.0f, 32};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)]     = {2000.0f, 4000.0f, 64};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::LONG)]       = {4000.0f, 8000.0f, 320};
    _distanceSpeedProfiles[static_cast<int>(DistanceType::VERY_LONG)]  = {8000.0f, 16000.0f, 640};
}

PositionController::~PositionController()
{
    stop();
    disable();
    _instances[_motorId] = nullptr;
}

// -----------------------------
// Init / Enable
// -----------------------------
bool PositionController::begin()
{
    if (_initialized)
        return true;

    _stepper.setEnablePin(_enPin);
    _stepper.setPinsInverted(false, false, true);  // invert enable
    _stepper.disableOutputs();                     // start disabled

    if (!_dirMultiplexer.begin())
    {
        log_e("Failed to init DirMux for motor %d", _motorId + 1);
        return false;
    }
    if (!_dirMultiplexer.selectMotor(_motorId))
    {
        log_e("Failed to select motor %d in DirMux", _motorId + 1);
        return false;
    }

    const int idx = static_cast<int>(MovementType::MEDIUM_RANGE);
    _stepper.setMaxSpeed(_speedProfiles[idx].maxSpeed);
    _stepper.setAcceleration(_speedProfiles[idx].acceleration);

    setCurrentPosition(0);

    _initialized = true;
    log_d("Motor %d initialized", _motorId + 1);
    return true;
}

void PositionController::setCurrentPosition(int32_t positionSteps)
{
    _stepper.setCurrentPosition(static_cast<long>(positionSteps));
    _status.currentSteps = positionSteps;
}

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

// -----------------------------
// Positioning API
// -----------------------------
bool PositionController::moveToSteps(int32_t targetSteps, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    const int32_t current = getCurrentSteps();
    const int32_t delta   = targetSteps - current;
    const int32_t dist    = std::abs(delta);

    if (!isValidMovementDistanceSteps(dist))
    {
        log_w("Motor %d: distance %d < 6 steps → ignore", _motorId + 1, dist);
        return false;
    }

    MovementCommand cmd{};
    cmd.motorId               = _motorId;
    cmd.targetSteps           = targetSteps;
    cmd.movementType          = movementType;
    cmd.distanceType          = calculateDistanceTypeSteps(dist);
    cmd.relative              = false;
    cmd.priority              = 1;
    cmd.controlMode           = controlMode;
    cmd.movementDistanceSteps = dist;
    return queueMovementCommand(cmd);
}

bool PositionController::moveRelativeSteps(int32_t deltaSteps, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    const int32_t dist = std::abs(deltaSteps);
    if (!isValidMovementDistanceSteps(dist))
    {
        log_w("Motor %d: distance %d < 6 steps → ignore", _motorId + 1, dist);
        return false;
    }

    MovementCommand cmd{};
    cmd.motorId               = _motorId;
    cmd.targetSteps           = getCurrentSteps() + deltaSteps;
    cmd.movementType          = movementType;
    cmd.distanceType          = calculateDistanceTypeSteps(dist);
    cmd.relative              = true;
    cmd.priority              = 1;
    cmd.controlMode           = controlMode;
    cmd.movementDistanceSteps = dist;
    return queueMovementCommand(cmd);
}

void PositionController::stop()
{
    if (!_initialized)
        return;

    _stepper.stop();
    _stepper.setCurrentPosition(_stepper.currentPosition());

    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        _status.isMoving          = false;
        _status.totalMovementTime = now_ms() - _status.movementStartTime;
        xSemaphoreGive(_statusMutex);
    }
    log_i("Motor %d stopped", _motorId + 1);
}

bool PositionController::isMoving() const
{
    // AccelStepper::distanceToGo() is non‑const → use const_cast safely
    auto& stepper = const_cast<AccelStepper&>(_stepper);
    return _status.isMoving && (stepper.distanceToGo() != 0);
}

bool PositionController::isAtTarget() const
{
    if (!_enabled || !_initialized)
        return false;
    const auto&   stepper = const_cast<AccelStepper&>(_stepper);
    const int32_t cur     = static_cast<int32_t>(stepper.currentPosition());
    const int32_t tgt     = static_cast<int32_t>(stepper.targetPosition());
    return std::abs(cur - tgt) <= POSITION_ACCURACY_STEPS;
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
    return static_cast<int32_t>(const_cast<AccelStepper&>(_stepper).targetPosition());
}

MotorStatus PositionController::getMotorStatus()
{
    MotorStatus s  = _status;
    s.currentSteps = getCurrentSteps();
    s.targetSteps  = getTargetSteps();
    s.isMoving     = isMoving();
    s.isEnabled    = isEnabled();
    return s;
}

void PositionController::setMaxSpeed(float v)
{
    if (_initialized)
        _stepper.setMaxSpeed(v);
}
void PositionController::setAcceleration(float a)
{
    if (_initialized)
        _stepper.setAcceleration(a);
}

void PositionController::setSpeedProfile(MovementType t, float maxSpeed, float accel)
{
    const int idx = static_cast<int>(t);
    if (idx >= 0 && idx < 3)
    {
        _speedProfiles[idx].maxSpeed     = maxSpeed;
        _speedProfiles[idx].acceleration = accel;
    }
}

// -----------------------------
// RTOS task management
// -----------------------------
void PositionController::startPositionControlTask()
{
    if (_positionControlTaskHandle)
        return;

    if (!_movementCommandQueue)
    {
        _movementCommandQueue = xQueueCreate(10, sizeof(MovementCommand));
        if (!_movementCommandQueue)
        {
            log_e("Failed to create command queue");
            return;
        }
    }
    if (!_statusMutex)
    {
        _statusMutex = xSemaphoreCreateMutex();
        if (!_statusMutex)
        {
            log_e("Failed to create status mutex");
            return;
        }
    }

    const BaseType_t res = xTaskCreatePinnedToCore(positionControlTask, "PositionControl", 4096, nullptr, 3, &_positionControlTaskHandle, 1);
    if (res == pdPASS)
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
    if (_positionControlTaskHandle)
    {
        vTaskDelete(_positionControlTaskHandle);
        _positionControlTaskHandle = nullptr;
    }
    if (_movementCommandQueue)
    {
        vQueueDelete(_movementCommandQueue);
        _movementCommandQueue = nullptr;
    }
    if (_statusMutex)
    {
        vSemaphoreDelete(_statusMutex);
        _statusMutex = nullptr;
    }
    log_d("Position control task stopped");
}

bool PositionController::queueMovementCommand(const MovementCommand& c)
{
    if (!_movementCommandQueue)
        return false;
    return xQueueSend(_movementCommandQueue, &c, pdMS_TO_TICKS(100)) == pdTRUE;
}

// -----------------------------
// Internals
// -----------------------------
void PositionController::updateStatus()
{
    if (!_initialized)
        return;
    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        _status.currentSteps = static_cast<int32_t>(_stepper.currentPosition());
        _status.isMoving     = (const_cast<AccelStepper&>(_stepper).distanceToGo() != 0);
        xSemaphoreGive(_statusMutex);
    }
}

void PositionController::configureSpeedProfile(MovementType t)
{
    const int idx = static_cast<int>(t);
    if (idx >= 0 && idx < 3)
    {
        _stepper.setMaxSpeed(_speedProfiles[idx].maxSpeed);
        _stepper.setAcceleration(_speedProfiles[idx].acceleration);
    }
}

void PositionController::setSpeedForMovement(MovementType t)
{
    configureSpeedProfile(t);
    _status.lastMovementType = t;
}

void PositionController::setDirection(bool dir)
{
    if (!_initialized)
        return;
    _dirMultiplexer.setMotorDirection(_motorId, dir);
}

bool PositionController::executeMovement(const MovementCommand& cmd)
{
    if (!_enabled || !_initialized)
        return false;

    setControlMode(cmd.controlMode);
    const int32_t target = cmd.targetSteps;

    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        _status.targetSteps       = target;
        _status.isMoving          = true;
        _status.movementStartTime = now_ms();
        _status.lastMovementType  = cmd.movementType;
        _status.controlMode       = cmd.controlMode;
        xSemaphoreGive(_statusMutex);
    }

    configureSpeedForDistanceSteps(cmd.movementDistanceSteps);
    _stepper.moveTo(static_cast<long>(target));

    return true;
}

float PositionController::calculateOptimalSpeedSteps(int32_t distanceSteps, MovementType type)
{
    const float base = _speedProfiles[static_cast<int>(type)].maxSpeed;
    if (distanceSteps < 640)
        return base * 0.5f;
    else if (distanceSteps < 5760)
        return base * 0.8f;
    else
        return base;
}

void PositionController::positionControlTask(void* /*parameter*/)
{
    MovementCommand  cmd{};
    TickType_t       lastWake = xTaskGetTickCount();
    const TickType_t period   = pdMS_TO_TICKS(10);  // 10 ms

    log_d("Position control task running");

    for (;;)
    {
        if (_movementCommandQueue && xQueueReceive(_movementCommandQueue, &cmd, 0) == pdTRUE)
        {
            if (cmd.motorId < 4 && _instances[cmd.motorId])
            {
                _instances[cmd.motorId]->executeMovement(cmd);
            }
        }

        for (int i = 0; i < 4; ++i)
        {
            if (_instances[i])
            {
                _instances[i]->runPositionControl();
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&lastWake, period);
    }
}

void PositionController::runPositionControl()
{
    if (!_enabled || !_initialized)
        return;

    auto&      stepper = _stepper;
    const long dtg     = stepper.distanceToGo();
    if (dtg != 0)
    {
        setDirection(dtg > 0);
    }

    stepper.run();

    if (_status.isMoving && isHybridModeEnabled())
    {
        applyHybridModeCorrection();
    }

    // Update status every tick (cheap) to avoid shared static timer between instances
    updateStatus();

    if (_status.isMoving && const_cast<AccelStepper&>(stepper).distanceToGo() == 0)
    {
        if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            _status.isMoving          = false;
            _status.totalMovementTime = now_ms() - _status.movementStartTime;
            xSemaphoreGive(_statusMutex);
        }
        _movementCompleteFlag = true;
        log_i("Motor %d reached target", _motorId + 1);
    }
}

ControlMode PositionController::getControlMode() const
{
    return _controlMode;
}

bool PositionController::isHybridModeEnabled() const
{
    // Keep original semantics; avoid repeated heavy work here if possible
    return _controlMode == ControlMode::HYBRID && _encoderMae3 != nullptr && (_encoderMae3->enable() == mae3::Status::Ok);
}

EncoderState PositionController::getEncoderState() const
{
    if (_encoderMae3 != nullptr)
    {
        if (_encoderMae3->enable() == mae3::Status::Ok)
        {
            std::double_t pos{0U}, ton{0U}, toff{0U};
            if (_encoderMae3->tryGetPosition(pos, ton, toff))
            {
                return {ton, toff, 0, pos, 0, Direction::UNKNOWN};
            }
        }
    }
    return {0, 0, 0, 0, 0, Direction::UNKNOWN};
}

int32_t PositionController::getEncoderSteps()
{
    const std::double_t pulse_f = getEncoderState().position_pulse;
    return UnitConverter::convertFromPulses(pulse_f).TO_STEPS;
}

// ---- Global helpers
void initializePositionControllers()
{
    PositionController::startPositionControlTask();
}

void startPositionControlSystem()
{
    for (int i = 0; i < 4; ++i)
        if (PositionController::_instances[i])
            PositionController::_instances[i]->enable();
}

void stopPositionControlSystem()
{
    for (int i = 0; i < 4; ++i)
        if (PositionController::_instances[i])
        {
            PositionController::_instances[i]->stop();
            PositionController::_instances[i]->disable();
        }
    PositionController::stopPositionControlTask();
}

void PositionController::applyHybridModeCorrection()
{
    if (!isHybridModeEnabled() || !_status.isMoving)
        return;
    // Kept behavior: sample once at start in setControlMode(); no continuous correction here.
}

void PositionController::setControlMode(ControlMode mode)
{
    _controlMode = mode;
    switch (mode)
    {
        case ControlMode::OPEN_LOOP:
            _status.controlMode = ControlMode::OPEN_LOOP;
            break;
        case ControlMode::HYBRID:
            if (_encoderMae3 != nullptr && (_encoderMae3->enable() == mae3::Status::Ok))
            {
                _status.controlMode    = ControlMode::HYBRID;
                const int32_t encSteps = getEncoderSteps();
                _status.encoderSteps   = encSteps;
                log_d("Motor %d: HYBRID start at encoder=%d steps", _motorId + 1, encSteps);
            }
            else
            {
                log_w("Motor %d: HYBRID unavailable → OPEN_LOOP", _motorId + 1);
                _controlMode        = ControlMode::OPEN_LOOP;
                _status.controlMode = ControlMode::OPEN_LOOP;
            }
            break;
    }
}

// Distance helpers (unchanged semantics)
DistanceType PositionController::calculateDistanceTypeSteps(int32_t d)
{
    if (d < 6)
        return DistanceType::NEGLIGIBLE;
    if (d < 32)
        return DistanceType::VERY_SHORT;
    if (d < 64)
        return DistanceType::SHORT;
    if (d < 320)
        return DistanceType::MEDIUM;
    if (d < 640)
        return DistanceType::LONG;
    return DistanceType::VERY_LONG;
}

bool PositionController::isValidMovementDistanceSteps(int32_t d)
{
    return d >= 6;
}

void PositionController::setDistanceBasedSpeedProfile(DistanceType t)
{
    if (t == DistanceType::NEGLIGIBLE)
        return;
    const int idx = static_cast<int>(t);
    if (idx >= 0 && idx < 6)
    {
        _stepper.setMaxSpeed(_distanceSpeedProfiles[idx].maxSpeed);
        _stepper.setAcceleration(_distanceSpeedProfiles[idx].acceleration);
    }
}

float PositionController::calculateOptimalSpeedForDistanceSteps(int32_t d)
{
    const DistanceType t = calculateDistanceTypeSteps(d);
    if (t == DistanceType::NEGLIGIBLE)
        return 0.0f;
    const int idx = static_cast<int>(t);
    if (idx >= 0 && idx < 6)
    {
        const float base = _distanceSpeedProfiles[idx].maxSpeed;
        float       mul  = 1.0f;
        switch (t)
        {
            case DistanceType::VERY_SHORT:
                mul = 0.5f + (d - 6) * 0.5f / 26.0f;
                break;
            case DistanceType::SHORT:
                mul = 0.7f + (d - 32) * 0.3f / 32.0f;
                break;
            case DistanceType::MEDIUM:
                mul = 0.8f + (d - 64) * 0.2f / 256.0f;
                break;
            case DistanceType::LONG:
                mul = 0.9f + (d - 320) * 0.1f / 320.0f;
                break;
            case DistanceType::VERY_LONG:
                mul = 1.0f;
                break;
            default:
                mul = 1.0f;
                break;
        }
        return base * mul;
    }
    return _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)].maxSpeed;
}

float PositionController::calculateOptimalAccelerationForDistanceSteps(int32_t d)
{
    const DistanceType t = calculateDistanceTypeSteps(d);
    if (t == DistanceType::NEGLIGIBLE)
        return 0.0f;
    const int idx = static_cast<int>(t);
    if (idx >= 0 && idx < 6)
    {
        const float base = _distanceSpeedProfiles[idx].acceleration;
        float       mul  = 1.0f;
        switch (t)
        {
            case DistanceType::VERY_SHORT:
                mul = 0.3f + (d - 6) * 0.4f / 26.0f;
                break;
            case DistanceType::SHORT:
                mul = 0.5f + (d - 32) * 0.3f / 32.0f;
                break;
            case DistanceType::MEDIUM:
                mul = 0.7f + (d - 64) * 0.2f / 256.0f;
                break;
            case DistanceType::LONG:
                mul = 0.8f + (d - 320) * 0.2f / 320.0f;
                break;
            case DistanceType::VERY_LONG:
                mul = 1.0f;
                break;
            default:
                mul = 1.0f;
                break;
        }
        return base * mul;
    }
    return _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)].acceleration;
}

void PositionController::configureSpeedForDistanceSteps(int32_t d)
{
    if (!isValidMovementDistanceSteps(d))
        return;
    const float v = calculateOptimalSpeedForDistanceSteps(d);
    const float a = calculateOptimalAccelerationForDistanceSteps(d);
    _stepper.setMaxSpeed(v);
    _stepper.setAcceleration(a);
    log_d("Motor %d: d=%d -> v=%.0f st/s, a=%.0f st/s^2", _motorId + 1, d, v, a);
}

void PositionController::attachOnComplete(void (*cb)())
{
    _onComplete = cb;
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

void PositionController::setMovementCompleteFlag(bool f)
{
    _movementCompleteFlag = f;
}

MotorType PositionController::getMotorType() const
{
    return (_motorId == 0) ? MotorType::LINEAR : MotorType::ROTATIONAL;
}

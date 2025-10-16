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

#define pc_print false

// ---- Small helpers
static inline uint32_t now_ms() noexcept
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Static member initialization
TaskHandle_t        PositionController::_positionControlTaskHandle         = nullptr;
QueueHandle_t       PositionController::_movementCommandQueue              = nullptr;
SemaphoreHandle_t   PositionController::_statusMutex                       = nullptr;
PositionController* PositionController::_instances[POSITION_CONTROL_COUNT] = {nullptr};

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
    _status.positionErrorSteps = 0;
    _status.encoderSteps       = 0;

    _instances[motorId] = this;

    _speedProfiles[static_cast<uint8_t>(MovementType::SHORT_RANGE)]  = {600.0f, 1200.0f};
    _speedProfiles[static_cast<uint8_t>(MovementType::MEDIUM_RANGE)] = {6000.0f, 12000.0f};
    _speedProfiles[static_cast<uint8_t>(MovementType::LONG_RANGE)]   = {16000.0f, 24000.0f};

    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::SHORT)]      = {800.0f, 1600.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::MEDIUM)]     = {3000.0f, 8000.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::LONG)]       = {8000.0f, 16000.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::VERY_LONG)]  = {16000.0f, 24000.0f};
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

    _speedProfiles[static_cast<uint8_t>(MovementType::SHORT_RANGE)]  = {600.0f, 1200.0f};
    _speedProfiles[static_cast<uint8_t>(MovementType::MEDIUM_RANGE)] = {6000.0f, 12000.0f};
    _speedProfiles[static_cast<uint8_t>(MovementType::LONG_RANGE)]   = {16000.0f, 24000.0f};

    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::NEGLIGIBLE)] = {0.0f, 0.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::VERY_SHORT)] = {500.0f, 1000.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::SHORT)]      = {800.0f, 1600.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::MEDIUM)]     = {3000.0f, 8000.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::LONG)]       = {8000.0f, 16000.0f};
    _distanceSpeedProfiles[static_cast<uint8_t>(DistanceType::VERY_LONG)]  = {16000.0f, 24000.0f};
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
        Serial.printf("--- Failed to init DirMux for motor[%d]\r\n", _motorId + 1);
        return false;
    }
    if (!_dirMultiplexer.selectMotor(_motorId))
    {
        Serial.printf("--- Failed to select motor[%d] in DirMux\r\n", _motorId + 1);
        return false;
    }

    const int idx = static_cast<int>(MovementType::MEDIUM_RANGE);
    _stepper.setMaxSpeed(_speedProfiles[idx].maxSpeed);
    _stepper.setAcceleration(_speedProfiles[idx].acceleration);

    setCurrentPosition(0);

    // 🔹 Print to check the correctness of the initialization:
    const size_t n = sizeof(_distanceSpeedProfiles) / sizeof(_distanceSpeedProfiles[0]);
    for (int i = 0; i < (int)n; i++)
    {
        Serial.printf("[DBG] Profile[%d]: vmax=%.0f, acc=%.0f\r\n", i, _distanceSpeedProfiles[i].maxSpeed, _distanceSpeedProfiles[i].acceleration);
    }

    _initialized = true;
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
    Serial.printf("Motor[%d] disabled\r\n", _motorId + 1);
}

bool PositionController::isEnabled() const
{
    return _enabled && _status.isEnabled;
}

// -----------------------------
// Positioning API
// -----------------------------
bool PositionController::moveToSteps(int32_t targetSteps, MovementType movementType)
{
    if (!_enabled || !_initialized)
    {
        Serial.printf("[WARN] M%d: moveToSteps aborted (_enabled=%d, _initialized=%d)\r\n", _motorId + 1, _enabled, _initialized);
        return false;
    }

    const int32_t current = getCurrentSteps();
    const int32_t delta   = targetSteps - current;
    const int32_t dist    = std::abs(delta);

#if pc_print
    Serial.printf("\r\n[DBG] M%d current=%ld, target=%ld, delta=%ld, dist=%ld\r\n", _motorId + 1, static_cast<long>(current), static_cast<long>(targetSteps), static_cast<long>(delta), static_cast<long>(dist));
#endif

    if (!isValidMovementDistanceSteps(dist))
    {
        Serial.printf("Motor[%d]: distance %d < %d steps → ignore\r\n", _motorId + 1, dist, VALID_MOVEMENT_DISTANCE_STEPS);
        return false;
    }

    MovementCommand cmd{};
    cmd.motorId               = _motorId;
    cmd.targetSteps           = targetSteps;
    cmd.movementType          = movementType;
    cmd.distanceType          = calculateDistanceTypeSteps(dist);
    cmd.relative              = false;
    cmd.priority              = 1;
    cmd.movementDistanceSteps = dist;

#if pc_print
    Serial.printf("\r\n[DBG] Queue movement: motorId=%d, targetSteps=%ld, dist=%ld, type=%d, distType=%d, rel=%d, prio=%d\r\n\r\n",
                  cmd.motorId,
                  static_cast<long>(cmd.targetSteps),
                  static_cast<long>(cmd.movementDistanceSteps),
                  static_cast<int>(cmd.movementType),
                  static_cast<int>(cmd.distanceType),
                  cmd.relative,
                  cmd.priority);
#endif

    return queueMovementCommand(cmd);
}

bool PositionController::moveRelativeSteps(int32_t deltaSteps, MovementType movementType)
{
    if (!_enabled || !_initialized)
        return false;

    const int32_t dist = std::abs(deltaSteps);
    if (!isValidMovementDistanceSteps(dist))
    {
        Serial.printf("Motor[%d]: distance %d < %d steps → ignore\r\n", _motorId + 1, dist, VALID_MOVEMENT_DISTANCE_STEPS);

        return false;
    }

    MovementCommand cmd{};
    cmd.motorId               = _motorId;
    cmd.targetSteps           = getCurrentSteps() + deltaSteps;
    cmd.movementType          = movementType;
    cmd.distanceType          = calculateDistanceTypeSteps(dist);
    cmd.relative              = true;
    cmd.priority              = 1;
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
    Serial.printf("[INFO] Motor%d stopped\r\n", _motorId + 1);
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

    auto&   stepper = _stepper;
    int32_t cur     = static_cast<int32_t>(stepper.currentPosition());

    // const auto&   stepper = const_cast<AccelStepper&>(_stepper);
    // const int32_t cur     = static_cast<int32_t>(stepper.currentPosition());
    const int32_t tgt = static_cast<int32_t>(stepper.targetPosition());
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
            Serial.printf("Failed to create command queue\r\n");
            return;
        }
    }
    if (!_statusMutex)
    {
        _statusMutex = xSemaphoreCreateMutex();
        if (!_statusMutex)
        {
            Serial.printf("Failed to create status mutex\r\n");
            return;
        }
    }

    const BaseType_t res = xTaskCreatePinnedToCore(positionControlTask, "PositionControl", RESOLUTION_STEPS, nullptr, 3, &_positionControlTaskHandle, 1);
    if (res == pdPASS)
    {
        // Serial.printf("Position control task started\r\n");
        esp_task_wdt_add(_positionControlTaskHandle);
    }
    else
    {
        Serial.printf("Failed to create position control task\r\n");
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
    Serial.printf("Position control task stopped\r\n");
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
void PositionController::updateStatus()  // amir2
{
    if (!_initialized)
        return;
    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        _status.currentSteps = static_cast<int32_t>(_stepper.currentPosition());
        //_status.isMoving     = (const_cast<AccelStepper&>(_stepper).distanceToGo() != 0);
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

    gpio_set_level(static_cast<gpio_num_t>(DriverPins::DIR[_motorId]), dir ? 1 : 0);  // amir temp
    //_dirMultiplexer.setMotorDirection(_motorId, dir);
}

static const char* toStr(MovementType t)
{
    switch (t)
    {
        case MovementType::SHORT_RANGE:
            return "SHORT";
        case MovementType::MEDIUM_RANGE:
            return "MEDIUM";
        case MovementType::LONG_RANGE:
            return "LONG";
        default:
            return "UNK";
    }
}

bool PositionController::executeMovement(const MovementCommand& cmd)
{
    const int32_t target = cmd.targetSteps;

#if pc_print
    Serial.printf("\r\n[DBG] M%d executeMovement: tgt=%ld, mType=%s, enabled=%d, init=%d\r\n", _motorId + 1, static_cast<long>(target), toStr(cmd.movementType), _enabled, _initialized);
#endif

    if (!_enabled || !_initialized)
    {
#if pc_print
        Serial.printf("[WARN] M%d executeMovement aborted: _enabled=%d, _initialized=%d ❌\r\n", _motorId + 1, _enabled, _initialized);
#endif
        return false;
    }

    if (!_enabled || !_initialized)
        return false;

    if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        _status.targetSteps       = target;
        _status.isMoving          = true;
        _status.movementStartTime = now_ms();
        _status.lastMovementType  = cmd.movementType;
        xSemaphoreGive(_statusMutex);
#if pc_print
        Serial.printf("\r\n[DBG] M%d status set: targetSteps=%ld, isMoving=1, lastType=%s, t0=%lu ms ✅\r\n", _motorId + 1, static_cast<long>(_status.targetSteps), toStr(_status.lastMovementType), static_cast<unsigned long>(_status.movementStartTime));
#endif
    }
    else
    {
#if pc_print
        Serial.printf("\r\n[ERR] M%d executeMovement: xSemaphoreTake(_statusMutex,50ms) FAILED ❗ (tgt=%ld)\r\n", _motorId + 1, static_cast<long>(target));
#endif
        return false;
    }
    // inja
    configureSpeedByDistanceSteps(cmd.movementDistanceSteps);
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
    const TickType_t period   = pdMS_TO_TICKS(1);  // was 10 -> now 1 ms

    // Serial.printf("Position control task running\r\n");

    for (;;)
    {
        if (_movementCommandQueue && xQueueReceive(_movementCommandQueue, &cmd, 0) == pdTRUE)
        {
#if pc_print
            Serial.printf("\r\n[DBG] Dequeued cmd: motorId=%d, tgt=%ld, dist=%ld, type=%d, distType=%d, rel=%d, prio=%d\r\n",
                          cmd.motorId,
                          static_cast<long>(cmd.targetSteps),
                          static_cast<long>(cmd.movementDistanceSteps),
                          static_cast<int>(cmd.movementType),
                          static_cast<int>(cmd.distanceType),
                          cmd.relative,
                          cmd.priority);
#endif
            if (cmd.motorId < POSITION_CONTROL_COUNT && _instances[cmd.motorId])
            {
#if pc_print
                Serial.printf("\r\n[DBG] Executing cmd for Motor%d...\r\n", cmd.motorId + 1);
#endif
                _instances[cmd.motorId]->executeMovement(cmd);
            }
            else
            {
#if pc_print
                Serial.printf("\r\n[WARN] Invalid motorId=%d or null instance — command skipped ❌\r\n", cmd.motorId);
#endif
            }
        }
        for (int k = 0; k < 7; ++k)  // 5x faster call to run()
        {
            for (int i = 0; i < POSITION_CONTROL_COUNT; ++i)
            {
                if (_instances[i])
                {
                    _instances[i]->runPositionControl();
                    _instances[i]->handleMovementComplete();
                }
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

    // Update status every tick (cheap) to avoid shared static timer between instances
    // Update status periodically
    static int32_t lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 100)  // Update every 100ms
    {
        updateStatus();  // amir2
        lastStatusUpdate = millis();
    }

    // Check if movement is complete
    if (_status.isMoving && const_cast<AccelStepper&>(stepper).distanceToGo() == 0)
    {
        if (xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            _status.isMoving          = false;
            _status.totalMovementTime = now_ms() - _status.movementStartTime;
            xSemaphoreGive(_statusMutex);
        }
        _movementCompleteFlag = true;
        Serial.printf("[INFO] Motor%d reached target\r\n", _motorId + 1);
    }
}

int32_t PositionController::getEncoderSteps()
{
    const std::double_t pulse_f = 333;  // getEncoderState().position_pulse;
    return UnitConverter::convertFromPulses(pulse_f).TO_STEPS;
}

// ---- Global helpers
void initializePositionControllers()
{
    PositionController::startPositionControlTask();
}

void startPositionControlSystem()
{
    for (int i = 0; i < POSITION_CONTROL_COUNT; ++i)
        if (PositionController::_instances[i])
            PositionController::_instances[i]->enable();
}

void stopPositionControlSystem()
{
    for (int i = 0; i < POSITION_CONTROL_COUNT; ++i)
        if (PositionController::_instances[i])
        {
            PositionController::_instances[i]->stop();
            PositionController::_instances[i]->disable();
        }
    PositionController::stopPositionControlTask();
}

// Distance helpers (unchanged semantics)
DistanceType PositionController::calculateDistanceTypeSteps(int32_t d)
{
    if (d < 1)
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
    return d >= VALID_MOVEMENT_DISTANCE_STEPS;
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

// If you don't have std::clamp, make your own style clamp:
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

float PositionController::calculateOptimalSpeedForDistanceSteps(int32_t d)
{
    const int32_t      dist = std::abs(d);  // ← Always work with absolute values
    const DistanceType t    = calculateDistanceTypeSteps(dist);
    if (t == DistanceType::NEGLIGIBLE)
        return 0.0f;

    // Actual size of the profile table
    const size_t nProfiles = sizeof(_distanceSpeedProfiles) / sizeof(_distanceSpeedProfiles[0]);
    const int    idx       = static_cast<int>(t);
    if (idx < 0 || idx >= static_cast<int>(nProfiles))
    {
        // safe fallback
        return _distanceSpeedProfiles[static_cast<int>(DistanceType::MEDIUM)].maxSpeed;
    }

    const float base = _distanceSpeedProfiles[idx].maxSpeed;
    float       mul  = 1.0f;

    // Note: a..b bounds must be the same as calculateDistanceTypeSteps logic
    switch (t)
    {
        case DistanceType::VERY_SHORT:
        {  // [6,32)
            const float a = 6.f, b = 32.f;
            mul = 0.5f + (dist - a) * (1.0f - 0.5f) / (b - a);
            break;
        }
        case DistanceType::SHORT:
        {  // [32,64)
            const float a = 32.f, b = 64.f;
            mul = 0.7f + (dist - a) * (1.0f - 0.7f) / (b - a);
            break;
        }
        case DistanceType::MEDIUM:
        {  // [64,320)
            const float a = 64.f, b = 320.f;
            mul = 0.8f + (dist - a) * (1.0f - 0.8f) / (b - a);
            break;
        }
        case DistanceType::LONG:
        {  // [320,640) — if this is the range in your classification
            const float a = 320.f, b = 640.f;
            mul = 0.9f + (dist - a) * (1.0f - 0.9f) / (b - a);
            break;
        }
        case DistanceType::VERY_LONG:
            mul = 1.0f;
            break;

        default:
            mul = 1.0f;
            break;
    }

    // Prevent out-of-range operation
    mul = clampf(mul, 0.0f, 1.0f);

    float v = base * mul;

    // If you have a global speed limit, apply it here
    // if (_maxAllowedSpeedStepsPerSec > 0) v = std::min(v, _maxAllowedSpeedStepsPerSec);

    // Optional debug:
    // Serial.printf("[DBG] M%d: d=%ld, type=%d, base=%.0f, mul=%.2f -> v=%.0f ​​st/s\r\n",
    // _motorId + 1, static_cast<long>(dist), idx, base, mul, v);

    return v;
}

float PositionController::calculateOptimalAccelerationForDistanceSteps(int32_t d)
{
    const int32_t      dist = std::abs(d);
    const DistanceType t    = calculateDistanceTypeSteps(dist);
    if (t == DistanceType::NEGLIGIBLE)
        return 0.0f;

    const size_t n   = sizeof(_distanceSpeedProfiles) / sizeof(_distanceSpeedProfiles[0]);
    const int    idx = (int)t;
    if (idx < 0 || idx >= (int)n)
        return _distanceSpeedProfiles[(int)DistanceType::MEDIUM].acceleration;

    const float base = _distanceSpeedProfiles[idx].acceleration;
    float       mul  = 1.0f;

    switch (t)
    {
        case DistanceType::VERY_SHORT:
        {
            const float a = 6.f, b = 32.f;
            mul = 0.5f + (dist - a) * (1.0f - 0.5f) / (b - a);
            break;
        }
        case DistanceType::SHORT:
        {
            const float a = 32.f, b = 64.f;
            mul = 0.7f + (dist - a) * (1.0f - 0.7f) / (b - a);
            break;
        }
        case DistanceType::MEDIUM:
        {
            const float a = 64.f, b = 320.f;
            mul = 0.8f + (dist - a) * (1.0f - 0.8f) / (b - a);
            break;
        }
        case DistanceType::LONG:
        {
            const float a = 320.f, b = 640.f;
            mul = 0.9f + (dist - a) * (1.0f - 0.9f) / (b - a);
            break;
        }
        case DistanceType::VERY_LONG:
        default:
            mul = 1.0f;
            break;
    }
    mul = clampf(mul, 0.0f, 1.0f);

    float a = base * mul;

    // Safe minimums
    const float minAccel = 1000.0f;  // Depending on the hardware
    if (a < minAccel)
        a = minAccel;

    return a;
}

void PositionController::configureSpeedByDistanceSteps(int32_t d)
{
    if (!isValidMovementDistanceSteps(d))
    {
        Serial.printf("[INFO] Motor%d: speed config skipped (movement %d < %d steps)\r\n", _motorId + 1, d, VALID_MOVEMENT_DISTANCE_STEPS);
        return;
    }

    float v = calculateOptimalSpeedForDistanceSteps(d);
    float a = calculateOptimalAccelerationForDistanceSteps(d);

    _stepper.setMaxSpeed(v);
    _stepper.setAcceleration(a);
#if pc_print
    Serial.printf("\r\n[INFO] Motor%d: d=%ld -> v=%.0f st/s, a=%.0f st/s^2\r\n", _motorId + 1, (long)d, v, a);
#endif
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

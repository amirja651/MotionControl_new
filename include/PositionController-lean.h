// =============================
// File: PositionController.h
// Lean, allocation-free, RT-friendly rewrite (ESP32 + Arduino)
// - Same public API / semantics as your current module
// - Smaller hot path, fewer branches, no dynamic allocation in RT loop
// - Logging compile-time gated; watchdog fed in task; encoder optional
// =============================
#ifndef POSITION_CONTROLLER_H
    #define POSITION_CONTROLLER_H

    #include <AccelStepper.h>
    #include <Arduino.h>
    #include <esp_timer.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/queue.h>
    #include <freertos/semphr.h>
    #include <freertos/task.h>

    #include "DirMultiplexer.h"
    #include "TMC5160Manager.h"
    #include "UnitConverter.h"
    #include "mae3_encoder.h"  // provides mae3::Mae3Encoder

    // ---- Compile-time switches
    #ifndef PC_LOG
        #define PC_LOG 1
    #endif

    #ifndef PC_TASK_CORE
        #define PC_TASK_CORE 1
    #endif
    #ifndef PC_TASK_PRIO
        #define PC_TASK_PRIO 3
    #endif
    #ifndef PC_TASK_STACK
        #define PC_TASK_STACK 4096
    #endif
    #ifndef PC_TICK_MS
        #define PC_TICK_MS 10
    #endif

    #if PC_LOG
        #define PC_LOGD(fmt, ...) log_d(fmt, ##__VA_ARGS__)
        #define PC_LOGI(fmt, ...) log_i(fmt, ##__VA_ARGS__)
        #define PC_LOGW(fmt, ...) log_w(fmt, ##__VA_ARGS__)
        #define PC_LOGE(fmt, ...) log_e(fmt, ##__VA_ARGS__)
    #else
        #define PC_LOGD(...) ((void)0)
        #define PC_LOGI(...) ((void)0)
        #define PC_LOGW(...) ((void)0)
        #define PC_LOGE(...) ((void)0)
    #endif

// ---- Types (kept compatible with your existing code) ----
enum class MovementType : uint8_t
{
    SHORT_RANGE  = 0,
    MEDIUM_RANGE = 1,
    LONG_RANGE   = 2
};
enum class DistanceType : uint8_t
{
    NEGLIGIBLE = 0,
    VERY_SHORT = 1,
    SHORT      = 2,
    MEDIUM     = 3,
    LONG       = 4,
    VERY_LONG  = 5
};
enum class ControlMode : uint8_t
{
    OPEN_LOOP = 0,
    HYBRID    = 1
};
enum class MotorType : uint8_t
{
    ROTATIONAL = 0,
    LINEAR     = 1
};
enum class Direction : uint8_t
{
    UNKNOWN = 0,
    FWD     = 1,
    REV     = 2
};

struct SpeedProfile
{
    float maxSpeed;
    float acceleration;
};
struct DistanceProfile
{
    float   maxSpeed;
    float   acceleration;
    int32_t minSteps;
};

struct EncoderState
{
    double    ton;  // kept for compatibility
    double    toff;
    int       error;
    double    position_pulse;
    int       velocity;
    Direction dir;
};

struct MotorStatus
{
    uint8_t      motorId;
    int32_t      currentSteps;
    int32_t      targetSteps;
    bool         isMoving;
    bool         isEnabled;
    MovementType lastMovementType;
    uint64_t     movementStartTime;  // us
    uint64_t     totalMovementTime;  // us
    ControlMode  controlMode;
    int32_t      positionErrorSteps;
    int32_t      encoderSteps;
};

struct MovementCommand
{
    uint8_t      motorId;
    int32_t      targetSteps;
    MovementType movementType;
    DistanceType distanceType;
    bool         relative;
    uint8_t      priority;
    ControlMode  controlMode;
    int32_t      movementDistanceSteps;
};

class PositionController
{
public:
    // Constructors (encoder optional)
    PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin, mae3::Mae3Encoder& encoderMae3) noexcept;

    PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin) noexcept;

    ~PositionController();

    // Init / enable
    bool begin();
    void enable();
    void disable();
    bool isEnabled() const;

    // Position commands
    bool moveToSteps(int32_t targetSteps, MovementType movementType, ControlMode controlMode);
    bool moveRelativeSteps(int32_t deltaSteps, MovementType movementType, ControlMode controlMode);
    void stop();
    bool isMoving() const;
    bool isAtTarget() const;

    // Position accessors
    void    setCurrentPosition(int32_t positionSteps);
    int32_t getCurrentSteps() const;
    int32_t getTargetSteps() const;

    // Profiles
    void setMaxSpeed(float speedStepsPerSec);
    void setAcceleration(float accelerationStepsPerSec2);
    void setSpeedProfile(MovementType type, float maxSpeed, float acceleration);

    // Status
    MotorStatus getMotorStatus();

    // Encoder / control mode
    ControlMode  getControlMode() const;
    void         setControlMode(ControlMode mode);
    EncoderState getEncoderState() const;
    int32_t      getEncoderSteps();

    // Task control (global)
    static void startPositionControlTask();
    static void stopPositionControlTask();

    // Queue command
    static bool queueMovementCommand(const MovementCommand& command);

    // Completion callback
    void attachOnComplete(void (*callback)());
    void handleMovementComplete();
    void setMovementCompleteFlag(bool flag);

    // Helpers
    static DistanceType calculateDistanceTypeSteps(int32_t distanceSteps);
    static bool         isValidMovementDistanceSteps(int32_t distanceSteps);

    MotorType getMotorType() const;

private:
    // Internals
    void updateStatus();
    void configureSpeedProfile(MovementType type);
    void setSpeedForMovement(MovementType type);
    void setDirection(bool direction);

    bool executeMovement(const MovementCommand& command);

    static void positionControlTask(void* parameter);
    void        runPositionControl();

    float        calculateOptimalSpeedSteps(int32_t distanceSteps, MovementType type);
    static float calculateOptimalSpeedForDistanceSteps(int32_t distanceSteps);
    static float calculateOptimalAccelerationForDistanceSteps(int32_t distanceSteps);
    static void  configureSpeedForDistanceStepsStatic(PositionController& pc, int32_t distanceSteps);
    void         configureSpeedForDistanceSteps(int32_t distanceSteps);

private:
    // Hardware & deps
    TMC5160Manager&    _driver;
    AccelStepper       _stepper;
    mae3::Mae3Encoder* _encoderMae3;  // optional
    DirMultiplexer&    _dirMultiplexer;

    // Pins / id
    uint16_t _stepPin;
    uint16_t _enPin;
    uint8_t  _motorId;

    // State
    MotorStatus _status;
    bool        _enabled;
    bool        _initialized;
    ControlMode _controlMode;
    bool        _movementCompleteFlag;
    void (*_onComplete)();
    bool _encoderReady;
    bool _lastDir;

    // Profiles
    SpeedProfile    _speedProfiles[3];
    DistanceProfile _distanceSpeedProfiles[6];

    // RTOS (shared)
    static TaskHandle_t        _positionControlTaskHandle;
    static QueueHandle_t       _movementCommandQueue;
    static SemaphoreHandle_t   _statusMutex;
    static PositionController* _instances[4];
};

#endif  // POSITION_CONTROLLER_H

// =============================
// File: PositionController.cpp
// =============================
#include "PositionController.h"
#include <cmath>
#include <esp_task_wdt.h>

// ---- Static member initialization ----
TaskHandle_t        PositionController::_positionControlTaskHandle = nullptr;
QueueHandle_t       PositionController::_movementCommandQueue      = nullptr;
SemaphoreHandle_t   PositionController::_statusMutex               = nullptr;
PositionController* PositionController::_instances[4]              = {nullptr};

// ---- Constructors ----
PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin, mae3::Mae3Encoder& encoderMae3) noexcept
    : _driver(driver),
      _stepper(AccelStepper::DRIVER, stepPin, 0 /*dir via mux*/),
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
      _onComplete(nullptr),
      _encoderReady(false),
      _lastDir(true)
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

    _speedProfiles[(int)MovementType::SHORT_RANGE]  = {2000.0f, 4000.0f};
    _speedProfiles[(int)MovementType::MEDIUM_RANGE] = {4000.0f, 8000.0f};
    _speedProfiles[(int)MovementType::LONG_RANGE]   = {8000.0f, 16000.0f};

    _distanceSpeedProfiles[(int)DistanceType::NEGLIGIBLE] = {0.0f, 0.0f, 0};
    _distanceSpeedProfiles[(int)DistanceType::VERY_SHORT] = {500.0f, 1000.0f, 6};
    _distanceSpeedProfiles[(int)DistanceType::SHORT]      = {1000.0f, 2000.0f, 32};
    _distanceSpeedProfiles[(int)DistanceType::MEDIUM]     = {2000.0f, 4000.0f, 64};
    _distanceSpeedProfiles[(int)DistanceType::LONG]       = {4000.0f, 8000.0f, 320};
    _distanceSpeedProfiles[(int)DistanceType::VERY_LONG]  = {8000.0f, 16000.0f, 640};
}

PositionController::PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin) noexcept : PositionController(motorId, driver, dirMultiplexer, stepPin, enPin, *reinterpret_cast<mae3::Mae3Encoder*>(nullptr))
{
    _encoderMae3 = nullptr;  // explicit no-encoder variant
}

PositionController::~PositionController()
{
    stop();
    disable();
    _instances[_motorId] = nullptr;
}

// ---- Init ----
bool PositionController::begin()
{
    if (_initialized)
        return true;

    _stepper.setEnablePin(_enPin);
    _stepper.setPinsInverted(false, false, true);  // invert enable
    _stepper.disableOutputs();

    if (!_dirMultiplexer.begin())
    {
        PC_LOGE("DIR mux init failed for motor %u", (unsigned)(_motorId + 1));
        return false;
    }
    if (!_dirMultiplexer.selectMotor(_motorId))
    {
        PC_LOGE("DIR mux select failed for motor %u", (unsigned)(_motorId + 1));
        return false;
    }

    // Default profile
    _stepper.setMaxSpeed(_speedProfiles[(int)MovementType::MEDIUM_RANGE].maxSpeed);
    _stepper.setAcceleration(_speedProfiles[(int)MovementType::MEDIUM_RANGE].acceleration);
    setCurrentPosition(0);

    // Prepare encoder availability once
    _encoderReady = (_encoderMae3 != nullptr) && (_encoderMae3->enable() == mae3::Status::Ok);

    _initialized = true;
    PC_LOGD("Motor %u initialized", (unsigned)(_motorId + 1));
    return true;
}

void PositionController::setCurrentPosition(int32_t positionSteps)
{
    _stepper.setCurrentPosition((long)positionSteps);
    _status.currentSteps = positionSteps;
}

// ---- Enable/Disable ----
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
    PC_LOGI("Motor %u disabled", (unsigned)(_motorId + 1));
}

bool PositionController::isEnabled() const
{
    return _enabled && _status.isEnabled;
}

// ---- Position commands ----
bool PositionController::moveToSteps(int32_t targetSteps, MovementType movementType, ControlMode controlMode)
{
    if (!_enabled || !_initialized)
        return false;

    const int32_t current = getCurrentSteps();
    const int32_t delta   = targetSteps - current;
    const int32_t dist    = (delta >= 0) ? delta : -delta;
    if (!isValidMovementDistanceSteps(dist))
    {
        PC_LOGW("Motor %u: negligible move %ld steps", (unsigned)(_motorId + 1), (long)dist);
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
    const int32_t dist = (deltaSteps >= 0) ? deltaSteps : -deltaSteps;
    if (!isValidMovementDistanceSteps(dist))
    {
        PC_LOGW("Motor %u: negligible move %ld steps", (unsigned)(_motorId + 1), (long)dist);
        return false;
    }
    const int32_t target = getCurrentSteps() + deltaSteps;

    MovementCommand cmd{};
    cmd.motorId               = _motorId;
    cmd.targetSteps           = target;
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

    if (_statusMutex && xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        _status.isMoving          = false;
        const uint64_t now        = (uint64_t)esp_timer_get_time();
        _status.totalMovementTime = now - _status.movementStartTime;
        xSemaphoreGive(_statusMutex);
    }
    PC_LOGI("Motor %u stopped", (unsigned)(_motorId + 1));
}

bool PositionController::isMoving() const
{
    return _status.isMoving && (_stepper.distanceToGo() != 0);
}

bool PositionController::isAtTarget() const
{
    if (!_enabled || !_initialized)
        return false;
    const int32_t     cur                     = (int32_t)_stepper.currentPosition();
    const int32_t     tgt                     = (int32_t)_stepper.targetPosition();
    constexpr int32_t POSITION_ACCURACY_STEPS = 1;  // keep tight
    return (cur > tgt ? cur - tgt : tgt - cur) <= POSITION_ACCURACY_STEPS;
}

int32_t PositionController::getCurrentSteps() const
{
    if (!_initialized)
        return 0;
    return (int32_t)const_cast<AccelStepper&>(_stepper).currentPosition();
}

int32_t PositionController::getTargetSteps() const
{
    if (!_initialized)
        return 0;
    return (int32_t)const_cast<AccelStepper&>(_stepper).targetPosition();
}

MotorStatus PositionController::getMotorStatus()
{
    MotorStatus s  = _status;
    s.currentSteps = getCurrentSteps();
    s.targetSteps  = getTargetSteps();
    setCurrentPosition(s.currentSteps);
    s.isMoving  = isMoving();
    s.isEnabled = isEnabled();
    return s;
}

// ---- Profiles ----
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
void PositionController::setSpeedProfile(MovementType t, float v, float a)
{
    const int i = (int)t;
    if (i >= 0 && i < 3)
    {
        _speedProfiles[i] = {v, a};
    }
}

// ---- Task management ----
void PositionController::startPositionControlTask()
{
    if (_positionControlTaskHandle)
        return;

    _movementCommandQueue = xQueueCreate(10, sizeof(MovementCommand));
    if (!_movementCommandQueue)
    {
        PC_LOGE("queue create failed");
        return;
    }

    _statusMutex = xSemaphoreCreateMutex();
    if (!_statusMutex)
    {
        PC_LOGE("status mutex create failed");
        return;
    }

    const BaseType_t ok = xTaskCreatePinnedToCore(positionControlTask, "PositionControl", PC_TASK_STACK, nullptr, PC_TASK_PRIO, &_positionControlTaskHandle, PC_TASK_CORE);
    if (ok == pdPASS)
    {
        PC_LOGD("Position control task started");
        esp_task_wdt_add(_positionControlTaskHandle);
    }
    else
    {
        PC_LOGE("Position control task create failed");
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
    PC_LOGD("Position control task stopped");
}

bool PositionController::queueMovementCommand(const MovementCommand& c)
{
    if (!_movementCommandQueue)
        return false;
    return xQueueSend(_movementCommandQueue, &c, pdMS_TO_TICKS(100)) == pdTRUE;
}

// ---- Private ----
void PositionController::updateStatus()
{
    if (!_initialized)
        return;
    if (_statusMutex && xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        _status.currentSteps = (int32_t)_stepper.currentPosition();
        _status.isMoving     = (_stepper.distanceToGo() != 0);
        xSemaphoreGive(_statusMutex);
    }
}

void PositionController::configureSpeedProfile(MovementType t)
{
    const int i = (int)t;
    if (i >= 0 && i < 3)
    {
        _stepper.setMaxSpeed(_speedProfiles[i].maxSpeed);
        _stepper.setAcceleration(_speedProfiles[i].acceleration);
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
    // Update only on change to reduce SPI/GPIO churn
    if (dir != _lastDir)
    {
        _dirMultiplexer.setMotorDirection(_motorId, dir);
        _lastDir = dir;
    }
    else
    {
        // Ensure correct motor is selected (cheap and safe)
        _dirMultiplexer.selectMotorUnchecked(_motorId);
    }
}

bool PositionController::executeMovement(const MovementCommand& cmd)
{
    if (!_enabled || !_initialized)
        return false;

    setControlMode(cmd.controlMode);

    const int32_t target = cmd.targetSteps;

    if (_statusMutex && xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        _status.targetSteps       = target;
        _status.isMoving          = true;
        _status.movementStartTime = (uint64_t)esp_timer_get_time();
        _status.lastMovementType  = cmd.movementType;
        _status.controlMode       = cmd.controlMode;
        xSemaphoreGive(_statusMutex);
    }

    configureSpeedForDistanceSteps(cmd.movementDistanceSteps);
    _stepper.moveTo((long)target);

    PC_LOGD("M%u -> %ld (dist=%ld, mode=%s)", (unsigned)(_motorId + 1), (long)target, (long)cmd.movementDistanceSteps, (cmd.controlMode == ControlMode::OPEN_LOOP ? "OPEN" : "HYBRID"));
    return true;
}

float PositionController::calculateOptimalSpeedSteps(int32_t distanceSteps, MovementType type)
{
    const float base = _speedProfiles[(int)type].maxSpeed;
    if (distanceSteps < 640)
        return base * 0.5f;
    if (distanceSteps < 5760)
        return base * 0.8f;
    return base;
}

void PositionController::positionControlTask(void* /*parameter*/)
{
    MovementCommand  cmd{};
    TickType_t       lastWake = xTaskGetTickCount();
    const TickType_t period   = pdMS_TO_TICKS(PC_TICK_MS);

    PC_LOGD("Position control task running");

    for (;;)
    {
        if (_movementCommandQueue && xQueueReceive(_movementCommandQueue, &cmd, 0) == pdTRUE)
        {
            if (cmd.motorId < 4 && _instances[cmd.motorId])
            {
                _instances[cmd.motorId]->executeMovement(cmd);
            }
        }

        for (int i = 0; i < 4; i++)
            if (auto* pc = _instances[i])
                pc->runPositionControl();

        esp_task_wdt_reset();
        vTaskDelayUntil(&lastWake, period);
    }
}

void PositionController::runPositionControl()
{
    if (!_enabled || !_initialized)
        return;

    const long d2g = _stepper.distanceToGo();
    if (d2g != 0)
    {
        const bool dir = d2g > 0;
        setDirection(dir);
    }

    _stepper.run();

    // Optional hybrid mode: lightweight single-sample correction at start only
    if (_status.isMoving && _controlMode == ControlMode::HYBRID && _encoderReady)
    {
        // no continuous correction in this lean version (same semantics as original)
    }

    static uint64_t lastUs = 0;
    const uint64_t  now    = (uint64_t)esp_timer_get_time();
    if ((now - lastUs) >= 100000 /*100 ms*/)
    {
        updateStatus();
        lastUs = now;
    }

    if (_status.isMoving && _stepper.distanceToGo() == 0)
    {
        if (_statusMutex && xSemaphoreTake(_statusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            _status.isMoving          = false;
            _status.totalMovementTime = now - _status.movementStartTime;
            xSemaphoreGive(_statusMutex);
        }
        _movementCompleteFlag = true;
        PC_LOGI("Motor %u reached target (%s)", (unsigned)(_motorId + 1), (_controlMode == ControlMode::OPEN_LOOP ? "OPEN-LOOP" : "HYBRID"));
    }
}

// ---- Control mode / encoder ----
ControlMode PositionController::getControlMode() const
{
    return _controlMode;
}

void PositionController::setControlMode(ControlMode m)
{
    _controlMode = m;
    switch (m)
    {
        case ControlMode::OPEN_LOOP:
            _status.controlMode = ControlMode::OPEN_LOOP;
            break;
        case ControlMode::HYBRID:
            _encoderReady = (_encoderMae3 != nullptr) && (_encoderMae3->enable() == mae3::Status::Ok);
            if (_encoderReady)
            {
                _status.controlMode  = ControlMode::HYBRID;
                const int32_t enc    = getEncoderSteps();
                _status.encoderSteps = enc;
                PC_LOGD("M%u HYBRID start enc=%ld", (unsigned)(_motorId + 1), (long)enc);
            }
            else
            {
                PC_LOGW("M%u: encoder not available -> OPEN", (unsigned)(_motorId + 1));
                _controlMode        = ControlMode::OPEN_LOOP;
                _status.controlMode = ControlMode::OPEN_LOOP;
            }
            break;
    }
}

EncoderState PositionController::getEncoderState() const
{
    if (_encoderMae3 && _encoderReady)
    {
        double pos = 0, ton = 0, toff = 0;
        if (_encoderMae3->tryGetPosition(pos, ton, toff))
        {
            return {ton, toff, 0, pos, 0, Direction::UNKNOWN};
        }
    }
    return
    {
        0, 0, 0, 0, 0, Direction
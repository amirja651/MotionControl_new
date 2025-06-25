#include "MotorSpeedController.h"

// Initialize static member
MotorSpeedController* MotorSpeedController::_motorInstances[MAX_MOTORS] = {nullptr};

// Static ISR handlers
void IRAM_ATTR MotorSpeedController::onTimerISR0()
{
    if (_motorInstances[0] && _motorInstances[0]->_enabled)
        _motorInstances[0]->onTimerISR();
}

void IRAM_ATTR MotorSpeedController::onTimerISR1()
{
    if (_motorInstances[1] && _motorInstances[1]->_enabled)
        _motorInstances[1]->onTimerISR();
}

void IRAM_ATTR MotorSpeedController::onTimerISR2()
{
    if (_motorInstances[2] && _motorInstances[2]->_enabled)
        _motorInstances[2]->onTimerISR();
}

void IRAM_ATTR MotorSpeedController::onTimerISR3()
{
    if (_motorInstances[3] && _motorInstances[3]->_enabled)
        _motorInstances[3]->onTimerISR();
}

MotorSpeedController::MotorSpeedController(uint8_t motorId, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN,
                                           uint16_t EN_PIN)
    : _driver(driver),
      _DIR_PIN(DIR_PIN),
      _STEP_PIN(STEP_PIN),
      _EN_PIN(EN_PIN),
      _motorId(motorId),
      _stepsRemaining(0),
      _moving(false),
      _speed(0),
      _targetPosition(0),
      _currentPosition(0),
      _timer(nullptr),
      _enabled(false),
      _onComplete(nullptr)
{
    _mux = portMUX_INITIALIZER_UNLOCKED;
    if (_motorId < MAX_MOTORS)
    {
        _motorInstances[_motorId] = this;
    }
    _movementCompleteFlag = false;
}

MotorSpeedController::~MotorSpeedController()
{
    disable();
    if (_motorId < MAX_MOTORS)
        _motorInstances[_motorId] = nullptr;
}

bool MotorSpeedController::begin()
{
    if (_motorId >= NUM_DRIVERS)
        return false;

    // Configure pins
    pinMode(_DIR_PIN, OUTPUT);
    pinMode(_STEP_PIN, OUTPUT);
    pinMode(_EN_PIN, OUTPUT);

    // Default state
    digitalWrite(_EN_PIN, HIGH);  // Disable driver initially
    digitalWrite(_DIR_PIN, LOW);
    digitalWrite(_STEP_PIN, LOW);

    // Initialize motor
    disable();

    // Store instance for interrupt handling
    _motorInstances[_motorId] = this;

    // Allocate timer
    attachInterruptHandler();

    return true;
}

void MotorSpeedController::enable()
{
    if (_enabled)
        return;

    _enabled = true;
    digitalWrite(_EN_PIN, LOW);
    attachInterruptHandler();
}

void MotorSpeedController::disable()
{
    if (!_enabled)
        return;

    _enabled = false;
    digitalWrite(_EN_PIN, HIGH);
    detachInterruptHandler();
}

void MotorSpeedController::attachInterruptHandler()
{
    // Allocate timer
    if (_timer == nullptr && _motorId < MAX_MOTORS)
    {
        _timer = timerBegin(_motorId, 80, true);  // 80 prescaler: 1us tick (80MHz/80)
        switch (_motorId)
        {
            case 0:
                timerAttachInterrupt(_timer, &onTimerISR0, false);
                break;
            case 1:
                timerAttachInterrupt(_timer, &onTimerISR1, false);
                break;
            case 2:
                timerAttachInterrupt(_timer, &onTimerISR2, false);
                break;
            case 3:
                timerAttachInterrupt(_timer, &onTimerISR3, false);
                break;
        }
        timerAlarmWrite(_timer, 1000000, true);  // Default 1Hz, will be set in move()
        timerAlarmDisable(_timer);
    }
}

void MotorSpeedController::detachInterruptHandler()
{
    if (_timer)
        timerDetachInterrupt(_timer);
    _timer = nullptr;
}

void MotorSpeedController::setDirection(bool forward)
{
    digitalWrite(_DIR_PIN, forward ? HIGH : LOW);
}

void MotorSpeedController::move(float position, float speed, float lastSpeed)
{
    if (_timer == nullptr || speed <= 0)
        return;
    int32_t steps = static_cast<int32_t>(position);  // position in steps
    if (steps == 0)
        return;

    portENTER_CRITICAL(&_mux);
    _stepsRemaining = abs(steps);
    _moving         = true;
    _speed          = speed;
    _targetPosition = position;
    portEXIT_CRITICAL(&_mux);

    setDirection(steps > 0);
    enable();

    if (lastSpeed != speed)
    {
        lastSpeed = speed;

        uint32_t interval_us = static_cast<uint32_t>(1e6f / speed);
        timerAlarmWrite(_timer, interval_us, true);
        timerAlarmEnable(_timer);
    }
}

void MotorSpeedController::stop()
{
    stopTimer();
    if (getMotorType() == MotorType::ROTATIONAL)
        disable();

    portENTER_CRITICAL(&_mux);
    _moving         = false;
    _stepsRemaining = 0;
    portEXIT_CRITICAL(&_mux);
}

void MotorSpeedController::stopTimer()
{
    if (_timer)
        timerAlarmDisable(_timer);
}

void MotorSpeedController::startTimer(uint32_t interval_us)
{
    if (_timer)
    {
        timerAlarmWrite(_timer, interval_us, true);
        timerAlarmEnable(_timer);
    }
}

void IRAM_ATTR MotorSpeedController::onTimerISR()
{
    portENTER_CRITICAL_ISR(&_mux);
    if (_stepsRemaining > 0)
    {
        digitalWrite(_STEP_PIN, HIGH);
        // Short pulse, must be >1us for most drivers
        __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
        digitalWrite(_STEP_PIN, LOW);
        _stepsRemaining--;
    }
    if (_stepsRemaining <= 0 && _moving)
    {
        timerAlarmDisable(_timer);
        _moving               = false;
        _movementCompleteFlag = true;
    }
    portEXIT_CRITICAL_ISR(&_mux);
}

void MotorSpeedController::attachOnComplete(void (*callback)())
{
    _onComplete = callback;
}

MotorType MotorSpeedController::getMotorType() const
{
    if (_motorId == 0)
        return MotorType::LINEAR;
    else
        return MotorType::ROTATIONAL;
}

float MotorSpeedController::wrapAngle180(float value)
{
    if (value > 180.0f)
        value -= 360.0f;
    else if (value < -180.0f)
        value += 360.0f;
    return value;
}

float MotorSpeedController::calculateSignedPositionError(float targetPos, float currentPos)
{
    float error = targetPos - currentPos;
    return error;
}

void MotorSpeedController::handleMovementComplete()
{
    if (_movementCompleteFlag)
    {
        _movementCompleteFlag = false;
        if (_onComplete)
            _onComplete();
    }
}

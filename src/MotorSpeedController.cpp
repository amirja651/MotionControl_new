#include "MotorSpeedController.h"

// Static array for ISR access
MotorSpeedController* MotorSpeedController::motorInstances[MAX_MOTORS] = {nullptr, nullptr, nullptr, nullptr};

// Static ISR handlers
void IRAM_ATTR MotorSpeedController::onTimerISR0()
{
    if (motorInstances[0])
        motorInstances[0]->onTimerISR();
}
void IRAM_ATTR MotorSpeedController::onTimerISR1()
{
    if (motorInstances[1])
        motorInstances[1]->onTimerISR();
}
void IRAM_ATTR MotorSpeedController::onTimerISR2()
{
    if (motorInstances[2])
        motorInstances[2]->onTimerISR();
}
void IRAM_ATTR MotorSpeedController::onTimerISR3()
{
    if (motorInstances[3])
        motorInstances[3]->onTimerISR();
}

MotorSpeedController::MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN,
                                           uint16_t EN_PIN)
    : _driver(driver),
      _DIR_PIN(DIR_PIN),
      _STEP_PIN(STEP_PIN),
      _EN_PIN(EN_PIN),
      _motorIndex(motorIndex),
      _motorEnabled(false),
      _stepsRemaining(0),
      _moving(false),
      _speed(0),
      _targetPosition(0),
      _currentPosition(0),
      _timer(nullptr),
      _onComplete(nullptr)
{
    _timerMux = portMUX_INITIALIZER_UNLOCKED;
    if (_motorIndex < MAX_MOTORS)
    {
        motorInstances[_motorIndex] = this;
    }
}

void MotorSpeedController::begin()
{
    if (_motorIndex >= NUM_DRIVERS)
        return;

    // Configure pins
    pinMode(_DIR_PIN, OUTPUT);
    pinMode(_STEP_PIN, OUTPUT);
    pinMode(_EN_PIN, OUTPUT);

    // Default state
    digitalWrite(_EN_PIN, HIGH);  // Disable driver initially
    digitalWrite(_DIR_PIN, LOW);
    digitalWrite(_STEP_PIN, LOW);

    // Initialize motor
    motorEnable(false);

    // Allocate timer
    if (_timer == nullptr && _motorIndex < MAX_MOTORS)
    {
        _timer = timerBegin(_motorIndex, 80, true);  // 80 prescaler: 1us tick (80MHz/80)
        switch (_motorIndex)
        {
            case 0:
                timerAttachInterrupt(_timer, &onTimerISR0, true);
                break;
            case 1:
                timerAttachInterrupt(_timer, &onTimerISR1, true);
                break;
            case 2:
                timerAttachInterrupt(_timer, &onTimerISR2, true);
                break;
            case 3:
                timerAttachInterrupt(_timer, &onTimerISR3, true);
                break;
        }
        timerAlarmWrite(_timer, 1000000, true);  // Default 1Hz, will be set in move()
        timerAlarmDisable(_timer);
    }
}

void MotorSpeedController::setDirection(bool forward)
{
    digitalWrite(_DIR_PIN, forward ? HIGH : LOW);
}

void MotorSpeedController::motorEnable(bool enable)
{
    _motorEnabled = enable;
    digitalWrite(_EN_PIN, enable ? LOW : HIGH);
}

bool MotorSpeedController::isMotorEnabled() const
{
    return _motorEnabled;
}

void MotorSpeedController::move(float position, float speed)
{
    if (_timer == nullptr || speed <= 0)
        return;
    int32_t steps = static_cast<int32_t>(position);  // position in steps
    if (steps == 0)
        return;

    portENTER_CRITICAL(&_timerMux);
    _stepsRemaining = abs(steps);
    _moving         = true;
    _speed          = speed;
    _targetPosition = position;
    portEXIT_CRITICAL(&_timerMux);

    setDirection(steps > 0);
    motorEnable(true);

    uint32_t interval_us = static_cast<uint32_t>(1e6f / speed);
    timerAlarmWrite(_timer, interval_us, true);
    timerAlarmEnable(_timer);
}

void MotorSpeedController::stop()
{
    stopTimer();
    motorEnable(false);
    portENTER_CRITICAL(&_timerMux);
    _moving         = false;
    _stepsRemaining = 0;
    portEXIT_CRITICAL(&_timerMux);
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
    portENTER_CRITICAL_ISR(&_timerMux);
    if (_stepsRemaining > 0)
    {
        digitalWrite(_STEP_PIN, HIGH);
        // Short pulse, must be >1us for most drivers
        __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
        digitalWrite(_STEP_PIN, LOW);
        _stepsRemaining--;
    }
    if (_stepsRemaining <= 0)
    {
        timerAlarmDisable(_timer);
        _moving = false;
        if (_onComplete)
            _onComplete();
    }
    portEXIT_CRITICAL_ISR(&_timerMux);
}

void MotorSpeedController::attachOnComplete(void (*callback)())
{
    _onComplete = callback;
}

MotorType MotorSpeedController::getMotorType() const
{
    if (_motorIndex == 0)
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
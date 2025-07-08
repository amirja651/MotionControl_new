#include "MotorSpeedController.h"

// Initialize static member
MotorSpeedController* MotorSpeedController::_motorInstances[4] = {nullptr};

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
void IRAM_ATTR MotorSpeedController::onTimerISR()  // amir 1402/04/21
{
    if (!_moving && _stepsRemaining <= 0)
        return;

    else if (_moving && _stepsRemaining <= 0)
    {
        stopTimer();
        return;
    }

    _tickCounter++;
    if (_tickCounter >= _ticksPerStep)
    {
        _tickCounter = 0;

        digitalWrite(_STEP_PIN, HIGH);
        __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");  // Short pulse, must be >1us for most drivers
        digitalWrite(_STEP_PIN, LOW);
        _stepsRemaining--;

        if (_stepsRemaining == 0)
            stopTimer();
    }
}

void MotorSpeedController::attachInterruptHandler()
{
    // Check if timer is already initialized
    if (_timer != nullptr)
    {
        Serial.println("Timer already attached, skipping...");
        return;
    }

    if (_motorId >= 4)
    {
        Serial.println("Invalid motor ID for timer");
        return;
    }

    // Create new timer instance
    _timer = timerBegin(_motorId, 80, true);  // 80 prescaler: 1us tick (80MHz/80)

    if (_timer == nullptr)
    {
        Serial.println("Failed to create timer!");
        return;
    }

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

    Serial.print("Timer attached for motor ");
    Serial.println(_motorId);
    __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
}
void MotorSpeedController::detachInterruptHandler()
{
    if (_timer == nullptr)
        return;

    // Stop timer first
    timerAlarmDisable(_timer);
    timerDetachInterrupt(_timer);

    // Delete timer instance
    timerEnd(_timer);
    _timer = nullptr;

    Serial.print("Timer detached for motor ");
    Serial.println(_motorId);
    __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
}

void MotorSpeedController::startTimer()
{
    if (_timer == nullptr)
        return;

    Serial.print("Timer started for motor ");
    Serial.println(_motorId);
    timerAlarmWrite(_timer, _timerTick_us, true);  // Default 10: 100KHz and 1000: 1KHz, will be set in move()
    timerAlarmEnable(_timer);
}
void MotorSpeedController::stopTimer()
{
    if (_timer == nullptr)
        return;

    timerAlarmDisable(_timer);
    _moving               = false;
    _movementCompleteFlag = true;
    Serial.print("Timer stopped for motor ");
    Serial.println(_motorId);
}

MotorSpeedController::MotorSpeedController(uint8_t motorId, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN, uint16_t EN_PIN)
    : _driver(driver),
      _DIR_PIN(DIR_PIN),
      _STEP_PIN(STEP_PIN),
      _EN_PIN(EN_PIN),
      _motorId(motorId),

      _stepsRemaining(0),
      _moving(false),
      _speed(0),
      _lastSpeed(0),
      _targetSpeed(0),
      _targetPosition(0),
      _currentPosition(0),

      _timer(nullptr),
      _enabled(false),

      _onComplete(nullptr),
      _movementCompleteFlag(false),

      _ticksPerStep(0),
      _tickCounter(0),

      _motorContext{}
{
    _mux                      = portMUX_INITIALIZER_UNLOCKED;
    _motorInstances[_motorId] = this;
}
MotorSpeedController::~MotorSpeedController()
{
    disable();
    stopTimer();
    detachInterruptHandler();
    _motorInstances[_motorId] = nullptr;
}

bool MotorSpeedController::begin()
{
    // Configure pins
    pinMode(_DIR_PIN, OUTPUT);
    pinMode(_STEP_PIN, OUTPUT);
    pinMode(_EN_PIN, OUTPUT);

    // Default state
    digitalWrite(_EN_PIN, HIGH);  // Disable driver initially
    digitalWrite(_DIR_PIN, HIGH);
    digitalWrite(_STEP_PIN, HIGH);

    // Initialize motor
    disable();

    // Store instance for interrupt handling
    _motorInstances[_motorId] = this;

    return true;
}

void MotorSpeedController::enable()
{
    if (_enabled)
        return;

    _enabled = true;
    digitalWrite(_EN_PIN, LOW);
    __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
}
void MotorSpeedController::disable()
{
    if (!_enabled)
        return;

    _enabled = false;
    digitalWrite(_EN_PIN, HIGH);
    __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
    Serial.print("Motor disabled: ");
    Serial.println(_motorId + 1);
}

bool MotorSpeedController::isEnabled() const
{
    return _enabled;
}
bool MotorSpeedController::isDisabled() const
{
    return !_enabled;
}

void MotorSpeedController::setDirection(bool forward)
{
    digitalWrite(_DIR_PIN, forward ? HIGH : LOW);
}
void MotorSpeedController::move(int32_t deltaPulsPosition, float targetSpeed, float lastSpeed)  // amir 1402/04/21
{
    if (_timer == nullptr || targetSpeed <= 0 || deltaPulsPosition == 0)
        return;

    _stepsRemaining = abs(deltaPulsPosition);
    _targetPosition = deltaPulsPosition;
    _targetSpeed    = targetSpeed;
    _lastSpeed      = lastSpeed;
    _moving         = true;

    if (lastSpeed != targetSpeed)
    {
        lastSpeed = targetSpeed;
        updateSpeedGradually();
        // setSpeed(targetSpeed);
    }
}
void MotorSpeedController::stop()
{
    stopTimer();
    detachInterruptHandler();
    _enabled        = false;
    _stepsRemaining = 0;
}

void MotorSpeedController::attachOnComplete(void (*callback)())
{
    _onComplete = callback;
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
float MotorSpeedController::calculateDegreesPositionError(float target, float current)
{
    float diff = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    return diff;
}
float MotorSpeedController::calculateSignedPositionError(float targetPos, float currentPos)
{
    float error = targetPos - currentPos;
    return error;
}

void MotorSpeedController::setTargetSpeed(float targetSpeed)
{
    _targetSpeed = targetSpeed;
}
void MotorSpeedController::setSpeed(float speedStepsPerSec)
{
    _speed        = speedStepsPerSec;
    _ticksPerStep = static_cast<uint32_t>(1e6f / (speedStepsPerSec * _timerTick_us));
}
void MotorSpeedController::updateSpeedGradually()
{
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate < 10)
        return;
    lastUpdate = millis();

    float currentSpeed = (_ticksPerStep == 0) ? 0 : 1e6f / (_ticksPerStep * _timerTick_us);
    float deltaSpeed   = _targetSpeed - currentSpeed;

    if (fabs(deltaSpeed) < 0.01f)
        return;

    float accel     = 1000.0f;        // steps/sec^2
    float speedStep = accel * 0.01f;  // because we update every 10ms

    if (deltaSpeed > 0)
        currentSpeed += speedStep;
    else
        currentSpeed -= speedStep;

    setSpeed(currentSpeed);
}

MotorContext& MotorSpeedController::getMotorContext() const
{
    _motorContext.stepsRemaining  = _stepsRemaining;
    _motorContext.moving          = _moving;
    _motorContext.speed           = _speed;
    _motorContext.lastSpeed       = _lastSpeed;
    _motorContext.targetSpeed     = _targetSpeed;
    _motorContext.targetPosition  = _targetPosition;
    _motorContext.currentPosition = _currentPosition;
    _motorContext.ticksPerStep    = _ticksPerStep;
    _motorContext.tickCounter     = _tickCounter;
    return _motorContext;
}
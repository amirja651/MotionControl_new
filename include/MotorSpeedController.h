#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "Pins.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <driver/timer.h>

enum class MotorType
{
    LINEAR     = 0,
    ROTATIONAL = 1,
};

struct MotorContext
{
    int32_t  stepsRemaining;
    bool     moving;
    float    speed;  // steps/sec
    float    lastSpeed;
    float    targetSpeed;
    float    targetPosition;
    float    currentPosition;
    uint32_t ticksPerStep;
    uint32_t tickCounter;
};

class MotorSpeedController
{
public:
    // Constructor
    MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN, uint16_t EN_PIN);
    ~MotorSpeedController();

    // Initialize the controller
    bool begin();

    void enable(bool force = false);
    void disable(bool force = false);

    bool isEnabled() const;
    bool isDisabled() const;

    void setDirection(bool forward);
    void move(int32_t deltaPulsPosition, float targetSpeed, float lastSpeed);  // position: steps, speed: steps/sec

    // Optional: attach a callback to be called when movement completes
    void attachOnComplete(void (*callback)());
    void handleMovementComplete();

    MotorType getMotorType() const;

    float wrapAngle180(float value);
    float calculateDegreesPositionError(float target, float current);
    float calculateSignedPositionError(float targetPos, float currentPos);

    void setTargetSpeed(float targetSpeed);
    void setSpeed(float speedStepsPerSec);
    void updateSpeedGradually();

    MotorContext& getMotorContext() const;

    void attachInterruptHandler();
    void detachInterruptHandler();

    void startTimer();
    void stopTimer();

private:
    // Hardware driver and pins
    TMC5160Manager& _driver;
    uint16_t        _DIR_PIN;
    uint16_t        _STEP_PIN;
    uint16_t        _EN_PIN;
    uint8_t         _motorId;

    // Stepper state
    volatile int32_t _stepsRemaining;
    volatile bool    _moving;
    float            _speed;  // steps/sec
    float            _lastSpeed;
    float            _targetSpeed;
    float            _targetPosition;
    float            _currentPosition;

    // Timer
    hw_timer_t*   _timer;
    volatile bool _enabled;
    // Optional movement complete callback
    void          (*_onComplete)();
    volatile bool _movementCompleteFlag;

    volatile uint32_t      _ticksPerStep;
    volatile uint32_t      _tickCounter;
    static constexpr float _timerTick_us = 10.0f;

    mutable MotorContext _motorContext;

    portMUX_TYPE                 _mux;
    static MotorSpeedController* _motorInstances[4];  // Timer/ISR support

    static void IRAM_ATTR onTimerISR0();
    static void IRAM_ATTR onTimerISR1();
    static void IRAM_ATTR onTimerISR2();
    static void IRAM_ATTR onTimerISR3();
    void IRAM_ATTR        onTimerISR();
};

#endif  // MOTOR_SPEED_CONTROLLER_H
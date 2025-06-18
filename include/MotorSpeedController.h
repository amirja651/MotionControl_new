#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "Pins.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <driver/timer.h>

// Maximum number of supported motors
constexpr uint8_t MAX_MOTORS = 4;

enum class MotorType
{
    LINEAR     = 0,
    ROTATIONAL = 1,
};

class MotorSpeedController
{
public:
    // Constructor
    MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN, uint16_t EN_PIN);

    // Initialize the controller
    void begin();
    void stop();
    void move(float position, float speed);  // position: steps, speed: steps/sec

    void setDirection(bool forward);
    void motorEnable(bool enable);
    bool isMotorEnabled() const;

    // Optional: attach a callback to be called when movement completes
    void attachOnComplete(void (*callback)());

    MotorType getMotorType() const;
    float     wrapAngle180(float value);
    float     calculateSignedPositionError(float targetPos, float currentPos);

    void handleMovementComplete();

private:
    // Timer/ISR support
    static MotorSpeedController* motorInstances[MAX_MOTORS];
    static void IRAM_ATTR        onTimerISR0();
    static void IRAM_ATTR        onTimerISR1();
    static void IRAM_ATTR        onTimerISR2();
    static void IRAM_ATTR        onTimerISR3();
    void IRAM_ATTR               onTimerISR();

    void startTimer(uint32_t interval_us);
    void stopTimer();

    // Hardware driver and pins
    TMC5160Manager& _driver;
    uint16_t        _DIR_PIN;
    uint16_t        _STEP_PIN;
    uint16_t        _EN_PIN;
    uint8_t         _motorIndex;
    bool            _motorEnabled;

    // Stepper state
    volatile int32_t _stepsRemaining;
    volatile bool    _moving;
    float            _speed;  // steps/sec
    float            _targetPosition;
    float            _currentPosition;

    // Timer
    hw_timer_t*  _timer;
    portMUX_TYPE _timerMux;

    // Optional movement complete callback
    void (*_onComplete)();

    volatile bool _movementCompleteFlag;
};

#endif  // MOTOR_SPEED_CONTROLLER_H
#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "Defines.h"
#include "Pins.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <driver/timer.h>

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
    ~MotorSpeedController();

    // Initialize the controller
    bool begin();
    void stop();
    void move(float position, float speed, float lastSpeed);  // position: steps, speed: steps/sec

    void setDirection(bool forward);
    void enable();
    void disable();

    inline bool isEnabled() const
    {
        return _enabled;
    }

    inline bool isDisabled() const
    {
        return !_enabled;
    }

    // Optional: attach a callback to be called when movement completes
    void attachOnComplete(void (*callback)());

    MotorType getMotorType() const;
    float     wrapAngle180(float value);
    float     calculateSignedPositionError(float targetPos, float currentPos);

    void handleMovementComplete();

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
    float            _targetPosition;
    float            _currentPosition;

    // Timer
    hw_timer_t*  _timer;
    portMUX_TYPE _mux;

    volatile bool _enabled;

    // Maximum number of supported motors
    static constexpr uint8_t MAX_MOTORS = NUM_DRIVERS;

    static MotorSpeedController* _motorInstances[MAX_MOTORS];  // Timer/ISR support

    // Optional movement complete callback
    void          (*_onComplete)();
    volatile bool _movementCompleteFlag;

    void attachInterruptHandler();
    void detachInterruptHandler();

#if NUM_DRIVERS > 0
    static void IRAM_ATTR onTimerISR0();
#endif
#if NUM_DRIVERS > 1
    static void IRAM_ATTR onTimerISR1();
#endif
#if NUM_DRIVERS > 2
    static void IRAM_ATTR onTimerISR2();
#endif
#if NUM_DRIVERS > 3
    static void IRAM_ATTR onTimerISR3();
#endif

    void IRAM_ATTR onTimerISR();

    void startTimer(uint32_t interval_us);
    void stopTimer();
};

#endif  // MOTOR_SPEED_CONTROLLER_H
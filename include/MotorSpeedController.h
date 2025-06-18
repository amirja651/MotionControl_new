#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "Pins.h"
#include "TMC5160Manager.h"
#include <Arduino.h>
#include <ESP32TimerInterrupt.h>

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
    void move(float position, float speed);

    inline MotorType getMotorType() const
    {
        if (_motorIndex == 0)
            return MotorType::LINEAR;
        else
            return MotorType::ROTATIONAL;
    }

    inline void setDirection(bool forward)
    {
        digitalWrite(_DIR_PIN, forward ? HIGH : LOW);
    }

    inline void motorEnable(bool enable)
    {
        _motorEnabled = enable;
        digitalWrite(_EN_PIN, enable ? LOW : HIGH);
    }

    inline bool isMotorEnabled() const
    {
        return _motorEnabled;
    }

    inline float wrapAngle180(float value)
    {
        if (value > 180.0f)
            value -= 360.0f;
        else if (value < -180.0f)
            value += 360.0f;
        return value;
    }

    inline float calculateSignedPositionError(float targetPos, float currentPos)
    {
        float error = targetPos - currentPos;
        return error;
    }

private:
    TMC5160Manager& _driver;

    uint16_t _DIR_PIN;
    uint16_t _STEP_PIN;
    uint16_t _EN_PIN;

    uint8_t _motorIndex;
    bool    _motorEnabled;
};

#endif  // MOTOR_SPEED_CONTROLLER_H
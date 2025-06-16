#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "Pins.h"
#include "TMC5160Manager.h"
#include <Arduino.h>

enum class MotorType
{
    LINEAR     = 0,
    ROTATIONAL = 1,
};

static constexpr uint8_t  LEDC_CHANNELS[NUM_DRIVERS] = {0, 1, 2, 3};
static constexpr uint8_t  LEDC_TIMER_BITS            = 10;
static constexpr uint32_t LEDC_BASE_FREQ             = 6000;  // 6kHz base frequency

class MotorSpeedController
{
public:
    // Constructor
    MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN, uint16_t EN_PIN);

    // Initialize the controller
    void begin();

    // Speed control methods
    void setSpeed(int16_t speed);  // Speed range: -100 to 100
    void stop();
    void emergencyStop();

    // Status methods
    int16_t getCurrentSpeed() const;

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

    inline bool isMotorMoving() const
    {
        return _isRunning;
    }

    inline float wrapAngle180(float value)
    {
        if (value > 180.0f)
            value -= 360.0f;
        else if (value < -180.0f)
            value += 360.0f;

        return value;
    }

    inline float calculateSignedPositionError(float target_pos, float current_pos)
    {
        float error = target_pos - current_pos;
        return error;
    }

    // Movement methods
    void moveForward();
    void moveBackward();
    void stopMotor();
    void setMotorDirection(bool direction);
    void updateMotorFrequency(float error_pulses, float target_position_pulses, float current_pos_pulses);

private:
    TMC5160Manager& _driver;

    uint16_t _DIR_PIN;
    uint16_t _STEP_PIN;
    uint16_t _EN_PIN;

    uint8_t _motorIndex;
    int16_t _currentSpeed;
    bool    _isRunning;
    bool    _motorEnabled;
    uint8_t _channel;

    // Helper methods
    inline void updatePWM(float frequency);
    float       calculateStoppingDistance(float current_freq);
    float       calculateFrequencyFromError(float error_pulses);
    void        logging(float base_freq, float error_pulses);
};

#endif  // MOTOR_SPEED_CONTROLLER_H
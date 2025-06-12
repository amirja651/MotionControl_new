#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "DriverPins.h"
#include "TMC5160Manager.h"
#include <Arduino.h>

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
    bool    isMoving() const;
    int16_t getCurrentSpeed() const;

    inline void setDirection(bool forward)
    {
        digitalWrite(_DIR_PIN, forward ? HIGH : LOW);
    }

    inline void driverEnable(bool enable)
    {
        driverEnabled = enable;
        digitalWrite(_EN_PIN, enable ? LOW : HIGH);
    }

    // Movement methods
    void moveForward();
    void moveBackward();
    void stopMotor();

private:
    TMC5160Manager& _driver;

    uint16_t _DIR_PIN;
    uint16_t _STEP_PIN;
    uint16_t _EN_PIN;

    uint8_t motorIndex;
    int16_t currentSpeed;
    bool    isRunning;
    bool    driverEnabled;

    static constexpr uint8_t  LEDC_CHANNELS[NUM_DRIVERS] = {0, 1, 2, 3};
    static constexpr uint8_t  LEDC_TIMER_BITS            = 10;
    static constexpr uint32_t LEDC_BASE_FREQ             = 5000;  // 5kHz base frequency

    // Helper methods
    void updatePWM();
};

#endif  // MOTOR_SPEED_CONTROLLER_H
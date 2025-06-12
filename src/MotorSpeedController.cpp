#include "MotorSpeedController.h"

// Define the static constexpr array
constexpr uint8_t MotorSpeedController::LEDC_CHANNELS[4];

MotorSpeedController::MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN,
                                           uint16_t EN_PIN)
    : _driver(driver),
      _DIR_PIN(DIR_PIN),
      _STEP_PIN(STEP_PIN),
      _EN_PIN(EN_PIN),
      motorIndex(motorIndex),
      currentSpeed(0),
      isRunning(false),
      driverEnabled(false)
{
}

void MotorSpeedController::begin()
{
    if (motorIndex >= NUM_DRIVERS)
        return;

    // Configure pins
    pinMode(_DIR_PIN, OUTPUT);
    pinMode(_STEP_PIN, OUTPUT);
    pinMode(_EN_PIN, OUTPUT);

    // Default state
    digitalWrite(_EN_PIN, HIGH);  // Disable driver initially
    digitalWrite(_DIR_PIN, LOW);
    digitalWrite(_STEP_PIN, LOW);

    // Configure LEDC
    ledcSetup(LEDC_CHANNELS[motorIndex], LEDC_BASE_FREQ, LEDC_TIMER_BITS);

    // Attach LEDC channels to step pins
    ledcAttachPin(DriverPins::STEP[motorIndex], LEDC_CHANNELS[motorIndex]);

    // Initialize motor
    driverEnable(false);
    stop();
}

void MotorSpeedController::setSpeed(int16_t speed)
{
    if (motorIndex >= NUM_DRIVERS)
        return;

    // Constrain speed to -100 to 100 range
    speed = constrain(speed, -100, 100);

    if (speed == 0)
    {
        stop();
        return;
    }

    // Set direction based on speed sign
    setDirection(speed > 0);

    // Convert speed to PWM value (0-255)
    uint32_t pwmValue = map(abs(speed), 0, 100, 0, 255);
    ledcWrite(LEDC_CHANNELS[motorIndex], pwmValue);

    currentSpeed = speed;
    isRunning    = true;
}

void MotorSpeedController::stop()
{
    if (motorIndex >= NUM_DRIVERS)
        return;

    ledcWrite(LEDC_CHANNELS[motorIndex], 0);
    currentSpeed = 0;
    isRunning    = false;
}

void MotorSpeedController::emergencyStop()
{
    stop();
    _driver.emergencyStop();
}

bool MotorSpeedController::isMoving() const
{
    return isRunning;
}

int16_t MotorSpeedController::getCurrentSpeed() const
{
    return currentSpeed;
}

void MotorSpeedController::updatePWM()
{
    if (motorIndex >= NUM_DRIVERS)
        return;

    if (isRunning)
    {
        uint32_t pwmValue = map(abs(currentSpeed), 0, 100, 0, 255);
        ledcWrite(LEDC_CHANNELS[motorIndex], pwmValue);
    }
}

void MotorSpeedController::moveForward() {}

void MotorSpeedController::moveBackward() {}

void MotorSpeedController::stopMotor() {}
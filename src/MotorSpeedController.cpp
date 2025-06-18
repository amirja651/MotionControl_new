#include "MotorSpeedController.h"
#include <ESP32TimerInterrupt.h>

MotorSpeedController::MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN,
                                           uint16_t EN_PIN)
    : _driver(driver), _DIR_PIN(DIR_PIN), _STEP_PIN(STEP_PIN), _EN_PIN(EN_PIN), _motorIndex(motorIndex), _motorEnabled(false)
{
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
}

void MotorSpeedController::stop()
{
    if (_motorIndex != (uint8_t)MotorType::LINEAR)
        motorEnable(false);
}

void MotorSpeedController::move(float position, float speed) {}
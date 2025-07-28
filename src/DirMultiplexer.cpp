#include "DirMultiplexer.h"
#include <esp_log.h>

// Truth table for 2-to-4 decoder (S1, S0 -> Y0, Y1, Y2, Y3)
// S1 S0 | Y0 Y1 Y2 Y3
//  0  0 |  1  0  0  0  (Motor 0)
//  0  1 |  0  1  0  0  (Motor 1)
//  1  0 |  0  0  1  0  (Motor 2)
//  1  1 |  0  0  0  1  (Motor 3)

DirMultiplexer::DirMultiplexer(uint8_t s0Pin, uint8_t s1Pin, uint8_t dirPin)
    : _s0Pin(s0Pin), _s1Pin(s1Pin), _dirPin(dirPin), _currentMotor(0), _currentDirection(false), _initialized(false)
{
}

bool DirMultiplexer::begin()
{
    if (_initialized)
        return true;

    // Configure GPIO pins
    pinMode(_s0Pin, OUTPUT);
    pinMode(_s1Pin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    // Initialize to safe state
    digitalWrite(_s0Pin, LOW);
    digitalWrite(_s1Pin, LOW);
    digitalWrite(_dirPin, LOW);

    _currentMotor     = 0;
    _currentDirection = false;
    _initialized      = true;

    log_i("DIR Multiplexer initialized - S0:%d, S1:%d, DIR:%d", _s0Pin, _s1Pin, _dirPin);
    return true;
}

bool DirMultiplexer::selectMotor(uint8_t motorId)
{
    if (!_initialized)
    {
        log_e("Multiplexer not initialized");
        return false;
    }

    if (!isValidMotorId(motorId))
    {
        log_e("Invalid motor ID: %d", motorId);
        return false;
    }

    setSelectPins(motorId);
    _currentMotor = motorId;

    log_d("Selected motor %d (S0:%d, S1:%d)", motorId, digitalRead(_s0Pin), digitalRead(_s1Pin));
    return true;
}

void DirMultiplexer::setDirection(bool direction)
{
    if (!_initialized)
    {
        log_e("Multiplexer not initialized");
        return;
    }

    digitalWrite(_dirPin, direction ? HIGH : LOW);
    _currentDirection = direction;

    log_d("Set direction %s for motor %d", direction ? "FORWARD" : "REVERSE", _currentMotor);
}

bool DirMultiplexer::setMotorDirection(uint8_t motorId, bool direction)
{
    if (!selectMotor(motorId))
        return false;

    setDirection(direction);
    return true;
}

uint8_t DirMultiplexer::getCurrentMotor() const
{
    return _currentMotor;
}

bool DirMultiplexer::getCurrentDirection() const
{
    return _currentDirection;
}

void DirMultiplexer::disable()
{
    if (!_initialized)
        return;

    // Set both select pins to HIGH to disable all outputs
    // (assuming active-low enable or high-impedance state)
    digitalWrite(_s0Pin, HIGH);
    digitalWrite(_s1Pin, HIGH);
    digitalWrite(_dirPin, LOW);

    log_d("Multiplexer disabled");
}

bool DirMultiplexer::test(bool printResults)
{
    if (!_initialized)
    {
        if (printResults)
            log_e("Multiplexer not initialized for testing");
        return false;
    }

    bool        testPassed = true;
    const char* testName   = "DIR Multiplexer Test";

    if (printResults)
        log_i("Starting %s", testName);

    // Test each motor channel
    for (uint8_t motorId = 0; motorId < 4; motorId++)
    {
        // Test forward direction
        if (!setMotorDirection(motorId, true))
        {
            testPassed = false;
            if (printResults)
                log_e("Test failed: Motor %d forward direction", motorId);
            continue;
        }

        // Verify selection
        if (getCurrentMotor() != motorId)
        {
            testPassed = false;
            if (printResults)
                log_e("Test failed: Motor selection verification for motor %d", motorId);
            continue;
        }

        // Test reverse direction
        if (!setMotorDirection(motorId, false))
        {
            testPassed = false;
            if (printResults)
                log_e("Test failed: Motor %d reverse direction", motorId);
            continue;
        }

        if (printResults)
            log_i("Motor %d: PASS", motorId);
    }

    // Test invalid motor ID
    if (setMotorDirection(4, true))
    {
        testPassed = false;
        if (printResults)
            log_e("Test failed: Should reject invalid motor ID 4");
    }

    // Test disable function
    disable();
    if (getCurrentMotor() != 0)  // Should remain at last valid selection
    {
        testPassed = false;
        if (printResults)
            log_e("Test failed: Disable function");
    }

    if (printResults)
    {
        if (testPassed)
            log_i("%s: PASSED", testName);
        else
            log_e("%s: FAILED", testName);
    }

    return testPassed;
}

void DirMultiplexer::setSelectPins(uint8_t motorId)
{
    // Extract S0 and S1 values from motor ID
    // motorId: 0=00, 1=01, 2=10, 3=11
    bool s0 = (motorId & 0x01) != 0;  // LSB
    bool s1 = (motorId & 0x02) != 0;  // MSB

    digitalWrite(_s0Pin, s0 ? HIGH : LOW);
    digitalWrite(_s1Pin, s1 ? HIGH : LOW);
}

bool DirMultiplexer::isValidMotorId(uint8_t motorId) const
{
    return motorId < 4;
}
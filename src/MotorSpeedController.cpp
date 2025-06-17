#include "MotorSpeedController.h"

MotorSpeedController::MotorSpeedController(uint8_t motorIndex, TMC5160Manager& driver, uint16_t DIR_PIN, uint16_t STEP_PIN,
                                           uint16_t EN_PIN)
    : _driver(driver),
      _DIR_PIN(DIR_PIN),
      _STEP_PIN(STEP_PIN),
      _EN_PIN(EN_PIN),
      _motorIndex(motorIndex),
      _currentSpeed(0),
      _isRunning(false),
      _motorEnabled(false),
      _channel(0)
{
}

void MotorSpeedController::begin()
{
    if (_motorIndex >= NUM_DRIVERS)
        return;

    _channel = LEDC_CHANNELS[_motorIndex];

    // Configure pins
    pinMode(_DIR_PIN, OUTPUT);
    pinMode(_STEP_PIN, OUTPUT);
    pinMode(_EN_PIN, OUTPUT);

    // Default state
    digitalWrite(_EN_PIN, HIGH);  // Disable driver initially
    digitalWrite(_DIR_PIN, LOW);
    digitalWrite(_STEP_PIN, LOW);

    // Configure LEDC
    ledcSetup(_channel, LEDC_BASE_FREQ, LEDC_TIMER_BITS);

    // Attach LEDC channels to step pins
    ledcAttachPin(_STEP_PIN, _channel);

    // Initialize motor
    motorEnable(false);
    stop();
}

void MotorSpeedController::setSpeed(int16_t speed)
{
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
    ledcWrite(_channel, pwmValue);

    _currentSpeed = speed;
    _isRunning    = true;
}

void MotorSpeedController::stop()
{
    ledcWrite(_channel, 0);
    _currentSpeed = 0;
    _isRunning    = false;
}

void MotorSpeedController::emergencyStop()
{
    stop();
    _driver.emergencyStop();
}

int16_t MotorSpeedController::getCurrentSpeed() const
{
    return _currentSpeed;
}

void MotorSpeedController::moveForward() {}

void MotorSpeedController::moveBackward() {}

void MotorSpeedController::setMotorDirection(bool direction)  // dir = true (Forward), false (Reverse)
{
    digitalWrite(_DIR_PIN, direction ? HIGH : LOW);
}

void MotorSpeedController::stopMotor()
{
    // Stop PWM output
    ledcWrite(_channel, 0);

    // 0. Stop all pulses immediately (very important)
    ledcWriteTone(_channel, 0);

    delayMicroseconds(100);  // A little pause until the last pulse is finished

    // 1. Final advance with higher current for precise stop
    // set_IHOLD_IRUN(index, 4, 25, 15);  // ≈48 % hold, ≈81 % run, 64 ms ramp

    // 2. If needed, give the final frequency to complete the rotation (e.g. 400 Hz)
    ledcWriteTone(_channel, 400);
    delay(2);  // How many milliseconds should we give until some real pulses are given

    // 3. Remove all pulses (complete stop)
    ledcWriteTone(_channel, 0);

    // 4. Wait for the mechanical body to relax
    delayMicroseconds(300);

    // 5. Set the final holding current (for a linear axis about 26% is enough)
    // set_IHOLD_IRUN(index, 4, 20, 8);  // ≈26 % hold, ≈65 % run, 64 ms ramp

    // 6. Disable the output if the motor is rotary
    if (_motorIndex != (uint8_t)MotorType::LINEAR)
        motorEnable(false);
}

inline void MotorSpeedController::updatePWM(float frequency)
{
    if (_isRunning)
    {
        ledcWriteTone(_channel, frequency);  // suitable for Stepper Motor
    }
    else
    {
        ledcWriteTone(_channel, 0);  // Stop pulses
    }
}

// This time → based on pulses
float MotorSpeedController::calculateStoppingDistance(float currentFreq)
{
    // We assume that at different speeds → how many pulses are needed to stop
    if (currentFreq <= 200)
        return 2;  // At low speed → 2 pulses are enough
    else if (currentFreq <= 500)
        return 4;
    else if (currentFreq <= 1000)
        return 6;
    else if (currentFreq <= 3000)
        return 10;
    else
        return 15;  // At high speed → we need up to 15 pulses of deceleration
}

#define ERROR_DEADBAND_PULSES 1  // less than 1 pulse → stop

float MotorSpeedController::calculateFrequencyFromError(float error)
{
    float abs_error = fabs(error);

    if (abs_error < ERROR_DEADBAND_PULSES)
        return 0;  // Full stop

    // mapping
    if (abs_error <= 2)
        return 200;  // Slow
    else if (abs_error <= 5)
        return 400;
    else if (abs_error <= 10)
        return 800;
    else if (abs_error <= 20)
        return 1500;
    else if (abs_error <= 50)
        return 3000;
    return 6000;
}

void MotorSpeedController::updateMotorFrequency(float error, float targetPos, float currentPos)
{
    static float lastFreq = 0;

    float baseFreq = calculateFrequencyFromError(error);

    // If we have to stop
    if (baseFreq == 0)
    {
        stopMotor();
        lastFreq = 0;
        return;
    }

    float distanceToTarget = fabs(targetPos - currentPos);
    float stoppingDistance = calculateStoppingDistance(lastFreq);

    // Parameters
    float maxFreqChangePerSec = 1000.0f;  // Hz/sec → Max acceleration/deceleration rate
    float minFreq             = 100.0f;   // Minimum frequency (avoid motor stall)
    float maxFreq             = 3000.0f;  // Maximum frequency
    float dt                  = 0.01f;    // Loop time in seconds → e.g. 10ms loop

    // Anticipate stopping → distance-based deceleration
    if (distanceToTarget <= stoppingDistance)
    {
        float reduction_factor = distanceToTarget / stoppingDistance;
        reduction_factor       = pow(reduction_factor, 1.5f);

        float targetFreq = max(minFreq, maxFreq * reduction_factor);
        baseFreq         = min(baseFreq, targetFreq);
    }

    // Limit rate of frequency change
    float maxFreqChange = maxFreqChangePerSec * dt;

    if (baseFreq > lastFreq)
    {
        baseFreq = min(baseFreq, lastFreq + maxFreqChange);
    }
    else
    {
        baseFreq = max(baseFreq, lastFreq - maxFreqChange);
    }

    // Apply min/max freq limits
    baseFreq = constrain(baseFreq, minFreq, maxFreq);

    // Save for next loop
    lastFreq = baseFreq;

    logging(baseFreq, error);

    // helper function:
    // updatePWM(base_freq);

    ledcWriteTone(_channel, baseFreq);  // suitable for Stepper Motor
}

void MotorSpeedController::logging(float baseFreq, float error)
{
    // Logging
    String buffer          = "ch: " + String(_channel) + " freq: " + String(baseFreq, 2);
    String bufferWithError = buffer + " err: " + String(error, 2);

    static String lastBuffer = "";
    if (buffer != lastBuffer)
    {
        Serial.print(bufferWithError);
        Serial.print(F("\r\n"));
        lastBuffer = buffer;
    }
}
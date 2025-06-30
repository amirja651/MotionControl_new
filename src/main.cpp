#include "Pins.h"
#include <Arduino.h>

// Interrupt-based PWM reading for maximum accuracy
volatile unsigned long lastRisingEdgeTime36 = 0, lastFallingEdgeTime36 = 0;
volatile unsigned long lastRisingEdgeTime39 = 0, lastFallingEdgeTime39 = 0;
volatile unsigned long pulseHigh36 = 0, pulseLow36 = 0;
volatile unsigned long pulseHigh39 = 0, pulseLow39 = 0;
volatile bool          newData36 = false, newData39 = false;

void IRAM_ATTR handleInterrupt36();
void IRAM_ATTR handleInterrupt39();

void          readPWMEncodersInterrupt();
void          readPWMEncoders();
unsigned long getMedian(unsigned long arr[], int size);
float         calculateStandardDeviation(unsigned long arr[], int size);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);

    delay(1000);
    while (!Serial)
        delay(10);

    // Configure pins 36 and 39 for digital PWM reading
    pinMode(36, INPUT);  // GPIO36 for PWM encoder 1
    pinMode(39, INPUT);  // GPIO39 for PWM encoder 2

    // Attach interrupts for maximum accuracy
    attachInterrupt(digitalPinToInterrupt(36), handleInterrupt36, CHANGE);
    attachInterrupt(digitalPinToInterrupt(39), handleInterrupt39, CHANGE);

    Serial.print(F("[Info][Setup] Digital pins 36 and 39 initialized with interrupts for PWM encoder reading\r\n"));

    // for disable all drivers pins - for avoid conflict in SPI bus
    // Initialize CS pins and turn them off
    for (uint8_t index = 0; index < 4; index++)
    {
        pinMode(DriverPins::CS[index], OUTPUT);
        digitalWrite(DriverPins::CS[index], HIGH);
    }

    Serial.print(F("\r\n"));
    Serial.flush();
}

void loop()
{
    // Read and print encoder voltages every 1000ms
    // readEncoderVoltages();

    // Read and print PWM encoder values every 1000ms
    // Choose between polling and interrupt-based methods:

    // Method 1: Optimized polling (good accuracy)
    readPWMEncodersInterrupt();

    // Method 2: Interrupt-based (maximum accuracy)
    // readPWMEncodersInterrupt();

    delay(1000);
}

// Interrupt handlers for precise timing
void IRAM_ATTR handleInterrupt36()
{
    unsigned long currentTime = micros();

    if (digitalRead(36) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime36 = currentTime;
        if (lastFallingEdgeTime36 != 0)
        {
            pulseLow36 = currentTime - lastFallingEdgeTime36;
        }
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime36 = currentTime;
        if (lastRisingEdgeTime36 != 0)
        {
            pulseHigh36 = currentTime - lastRisingEdgeTime36;
            newData36   = true;
        }
    }
}

void IRAM_ATTR handleInterrupt39()
{
    unsigned long currentTime = micros();

    if (digitalRead(39) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime39 = currentTime;
        if (lastFallingEdgeTime39 != 0)
        {
            pulseLow39 = currentTime - lastFallingEdgeTime39;
        }
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime39 = currentTime;
        if (lastRisingEdgeTime39 != 0)
        {
            pulseHigh39 = currentTime - lastRisingEdgeTime39;
            newData39   = true;
        }
    }
}

// Interrupt-based PWM reading method
void readPWMEncodersInterrupt()
{
    // Reset flags
    newData36 = false;
    newData39 = false;

    // Wait for new data (with timeout)
    unsigned long startWait = millis();
    while ((!newData36 || !newData39) && (millis() - startWait) < 100)
    {
        delayMicroseconds(10);
    }

    // Validate pulse widths according to MAE3 specifications
    const unsigned long MIN_PULSE_US = 50;
    const unsigned long MAX_PULSE_US = 4097;

    bool valid36 =
        (pulseHigh36 >= MIN_PULSE_US && pulseHigh36 <= MAX_PULSE_US && pulseLow36 >= MIN_PULSE_US && pulseLow36 <= MAX_PULSE_US);

    bool valid39 =
        (pulseHigh39 >= MIN_PULSE_US && pulseHigh39 <= MAX_PULSE_US && pulseLow39 >= MIN_PULSE_US && pulseLow39 <= MAX_PULSE_US);

    // Calculate position using MAE3 PWM formula for 12-bit
    int           position36 = 0, position39 = 0;
    unsigned long totalPeriod36 = pulseHigh36 + pulseLow36;
    unsigned long totalPeriod39 = pulseHigh39 + pulseLow39;

    if (valid36 && totalPeriod36 > 0)
    {
        position36 = ((pulseHigh36 * 4098) / totalPeriod36) - 1;
        position36 = constrain(position36, 0, 4095);
    }

    if (valid39 && totalPeriod39 > 0)
    {
        position39 = ((pulseHigh39 * 4098) / totalPeriod39) - 1;
        position39 = constrain(position39, 0, 4095);
    }

    // Convert to degrees
    float degrees36 = (position36 * 360.0f) / 4096.0f;
    float degrees39 = (position39 * 360.0f) / 4096.0f;

    // Print results
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│                            Interrupt-Based PWM Readings                    │\n");
    printf("├────────────────────────────────────────────────────────────────────────────┤\n");
    printf("│ Pin │ Position │ Degrees  │ High Pulse │ Low Pulse │ Total Period │ Status │\n");
    printf("├─────┼──────────┼──────────┼────────────┼───────────┼──────────────┼────────┤\n");
    printf("│ 36  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %s │\n", position36, degrees36, pulseHigh36, pulseLow36, totalPeriod36,
           valid36 ? "OK    " : "ERR   ");
    printf("│ 39  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %s │\n", position39, degrees39, pulseHigh39, pulseLow39, totalPeriod39,
           valid39 ? "OK    " : "ERR   ");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n");
}

// Optimized method for reading PWM encoder values with minimal error
void readPWMEncoders()
{
    // Constants for optimization
    const unsigned long TIMEOUT_US   = 5000;  // 5ms timeout
    const unsigned long MIN_PULSE_US = 50;    // Minimum valid pulse width
    const unsigned long MAX_PULSE_US = 4097;  // Maximum valid pulse width
    const int           SAMPLES      = 5;     // Number of samples for averaging

    // Arrays for multiple samples
    unsigned long pulseHigh36[SAMPLES] = {0}, pulseLow36[SAMPLES] = {0};
    unsigned long pulseHigh39[SAMPLES] = {0}, pulseLow39[SAMPLES] = {0};
    unsigned long totalPeriod36[SAMPLES] = {0}, totalPeriod39[SAMPLES] = {0};

    // Take multiple samples for averaging
    for (int sample = 0; sample < SAMPLES; sample++)
    {
        // Measure PWM for pin 36 with improved synchronization
        unsigned long startTime = micros();

        // Wait for stable signal - find falling edge
        while (digitalRead(36) == HIGH && (micros() - startTime) < TIMEOUT_US)
        {
            // Wait for falling edge
        }

        // Wait for rising edge
        startTime = micros();
        while (digitalRead(36) == LOW && (micros() - startTime) < TIMEOUT_US)
        {
            // Wait for rising edge
        }

        // Measure high pulse width with precise timing
        startTime               = micros();
        unsigned long highStart = startTime;
        while (digitalRead(36) == HIGH && (micros() - startTime) < TIMEOUT_US)
        {
            pulseHigh36[sample] = micros() - highStart;
        }

        // Measure low pulse width with precise timing
        startTime              = micros();
        unsigned long lowStart = startTime;
        while (digitalRead(36) == LOW && (micros() - startTime) < TIMEOUT_US)
        {
            pulseLow36[sample] = micros() - lowStart;
        }

        totalPeriod36[sample] = pulseHigh36[sample] + pulseLow36[sample];

        // Measure PWM for pin 39 with same improved method
        startTime = micros();
        while (digitalRead(39) == HIGH && (micros() - startTime) < TIMEOUT_US)
        {
            // Wait for falling edge
        }

        startTime = micros();
        while (digitalRead(39) == LOW && (micros() - startTime) < TIMEOUT_US)
        {
            // Wait for rising edge
        }

        startTime = micros();
        highStart = startTime;
        while (digitalRead(39) == HIGH && (micros() - startTime) < TIMEOUT_US)
        {
            pulseHigh39[sample] = micros() - highStart;
        }

        startTime = micros();
        lowStart  = startTime;
        while (digitalRead(39) == LOW && (micros() - startTime) < TIMEOUT_US)
        {
            pulseLow39[sample] = micros() - lowStart;
        }

        totalPeriod39[sample] = pulseHigh39[sample] + pulseLow39[sample];

        // Small delay between samples
        delayMicroseconds(100);
    }

    // Calculate median values to remove outliers
    unsigned long medianHigh36   = getMedian(pulseHigh36, SAMPLES);
    unsigned long medianLow36    = getMedian(pulseLow36, SAMPLES);
    unsigned long medianPeriod36 = medianHigh36 + medianLow36;

    unsigned long medianHigh39   = getMedian(pulseHigh39, SAMPLES);
    unsigned long medianLow39    = getMedian(pulseLow39, SAMPLES);
    unsigned long medianPeriod39 = medianHigh39 + medianLow39;

    // Validate pulse widths according to MAE3 specifications
    bool valid36 = (medianHigh36 >= MIN_PULSE_US && medianHigh36 <= MAX_PULSE_US && medianLow36 >= MIN_PULSE_US &&
                    medianLow36 <= MAX_PULSE_US && medianPeriod36 > 0 && medianPeriod36 <= 4098);

    bool valid39 = (medianHigh39 >= MIN_PULSE_US && medianHigh39 <= MAX_PULSE_US && medianLow39 >= MIN_PULSE_US &&
                    medianLow39 <= MAX_PULSE_US && medianPeriod39 > 0 && medianPeriod39 <= 4098);

    // Calculate position using MAE3 PWM formula for 12-bit with validation
    int position36 = 0, position39 = 0;

    if (valid36)
    {
        // x = ((t_on * 4098) / (t_on + t_off)) - 1
        position36 = ((medianHigh36 * 4098) / medianPeriod36) - 1;
        position36 = constrain(position36, 0, 4095);  // Clamp to valid range
    }

    if (valid39)
    {
        position39 = ((medianHigh39 * 4098) / medianPeriod39) - 1;
        position39 = constrain(position39, 0, 4095);  // Clamp to valid range
    }

    // Convert to degrees (360° / 4096 positions)
    float degrees36 = (position36 * 360.0f) / 4096.0f;
    float degrees39 = (position39 * 360.0f) / 4096.0f;

    // Calculate standard deviation for error estimation
    float stdDev36 = calculateStandardDeviation(pulseHigh36, SAMPLES);
    float stdDev39 = calculateStandardDeviation(pulseHigh39, SAMPLES);

    // Print results with error analysis
    printf("\n┌─────────────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│                              Optimized PWM Encoder Readings                          │\n");
    printf("├─────────────────────────────────────────────────────────────────────────────────────┤\n");
    printf("│ Pin │ Position │ Degrees  │ High Pulse │ Low Pulse │ Total Period │ StdDev │ Status │\n");
    printf("├─────┼──────────┼──────────┼────────────┼───────────┼──────────────┼────────┼────────┤\n");
    printf("│ 36  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %6.1f │ %s │\n", position36, degrees36, medianHigh36, medianLow36,
           medianPeriod36, stdDev36, valid36 ? "OK    " : "ERR   ");
    printf("│ 39  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %6.1f │ %s │\n", position39, degrees39, medianHigh39, medianLow39,
           medianPeriod39, stdDev39, valid39 ? "OK    " : "ERR   ");
    printf("└─────────────────────────────────────────────────────────────────────────────────────┘\n");
}

// Helper function to calculate median
unsigned long getMedian(unsigned long arr[], int size)
{
    // Create a copy for sorting
    unsigned long temp[size];
    memcpy(temp, arr, size * sizeof(unsigned long));

    // Sort the array
    for (int i = 0; i < size - 1; i++)
    {
        for (int j = 0; j < size - i - 1; j++)
        {
            if (temp[j] > temp[j + 1])
            {
                unsigned long t = temp[j];
                temp[j]         = temp[j + 1];
                temp[j + 1]     = t;
            }
        }
    }

    // Return median
    return temp[size / 2];
}

// Helper function to calculate standard deviation
float calculateStandardDeviation(unsigned long arr[], int size)
{
    if (size <= 1)
        return 0.0f;

    // Calculate mean
    unsigned long sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += arr[i];
    }
    float mean = (float)sum / size;

    // Calculate variance
    float variance = 0.0f;
    for (int i = 0; i < size; i++)
    {
        float diff = (float)arr[i] - mean;
        variance += diff * diff;
    }
    variance /= (size - 1);

    // Return standard deviation
    return sqrt(variance);
}

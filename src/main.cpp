#include "Pins.h"
#include <Arduino.h>

// Interrupt-based PWM reading for maximum accuracy
volatile unsigned long lastRisingEdgeTime36 = 0, lastFallingEdgeTime36 = 0;
volatile unsigned long lastRisingEdgeTime39 = 0, lastFallingEdgeTime39 = 0;
volatile unsigned long pulseHigh36 = 0, pulseLow36 = 0;
volatile unsigned long pulseHigh39 = 0, pulseLow39 = 0;
volatile bool          newData36 = false, newData39 = false;

// Statistics tracking for bypassed values
volatile unsigned long bypassedCount36 = 0, bypassedCount39 = 0;
volatile unsigned long totalInterrupts36 = 0, totalInterrupts39 = 0;

void IRAM_ATTR handleInterrupt36();
void IRAM_ATTR handleInterrupt39();

void          readPWMEncodersInterrupt();
void          readPWMEncoders();
unsigned long getMedian(unsigned long arr[], int size);
float         calculateStandardDeviation(unsigned long arr[], int size);
void          diagnoseEncoderSignals();
bool          validatePWMValues(unsigned long highPulse, unsigned long lowPulse, int pinNumber);

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

// Function to reset bypass statistics
void resetBypassStatistics()
{
    bypassedCount36   = 0;
    bypassedCount39   = 0;
    totalInterrupts36 = 0;
    totalInterrupts39 = 0;
    printf("Bypass statistics reset\n");
}

void loop()
{
    // Read and print encoder voltages every 1000ms
    // readEncoderVoltages();

    // Run diagnostics first (only once)
    static bool diagnosticsRun = false;
    if (!diagnosticsRun)
    {
        diagnoseEncoderSignals();
        diagnosticsRun = true;
    }

    // Reset statistics every 10 seconds
    static unsigned long lastResetTime = 0;
    if (millis() - lastResetTime > 10000)
    {
        resetBypassStatistics();
        lastResetTime = millis();
    }

    // Read and print PWM encoder values every 1000ms
    // Choose between polling and interrupt-based methods:

    // Method 1: Optimized polling (good accuracy)
    // readPWMEncoders();

    // Method 2: Interrupt-based (maximum accuracy)
    readPWMEncodersInterrupt();

    delay(1000);
}

// Interrupt handlers for precise timing
void IRAM_ATTR handleInterrupt36()
{
    unsigned long currentTime = micros();
    totalInterrupts36++;

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

            // Check total period before accepting the values
            unsigned long totalPeriod = pulseHigh36 + pulseLow36;

            // MAE3 standard: period should be around 4000 us (250 Hz)
            // Accept range: 2000-6000 us (167-500 Hz)
            if (totalPeriod >= 2000 && totalPeriod <= 6000)
            {
                newData36 = true;  // Accept valid data
            }
            else
            {
                // Bypass invalid data - reset values
                pulseHigh36 = 0;
                pulseLow36  = 0;
                newData36   = false;
                bypassedCount36++;
            }
        }
    }
}

void IRAM_ATTR handleInterrupt39()
{
    unsigned long currentTime = micros();
    totalInterrupts39++;

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

            // Check total period before accepting the values
            unsigned long totalPeriod = pulseHigh39 + pulseLow39;

            // MAE3 standard: period should be around 4000 us (250 Hz)
            // Accept range: 2000-6000 us (167-500 Hz)
            if (totalPeriod >= 2000 && totalPeriod <= 6000)
            {
                newData39 = true;  // Accept valid data
            }
            else
            {
                // Bypass invalid data - reset values
                pulseHigh39 = 0;
                pulseLow39  = 0;
                newData39   = false;
                bypassedCount39++;
            }
        }
    }
}

// Diagnostic function to check encoder signal quality
void diagnoseEncoderSignals()
{
    printf("\n=== ENCODER SIGNAL DIAGNOSTICS ===\n");

    // Check if interrupts are working
    printf("Interrupt Status:\n");
    printf("- Pin 36 interrupt attached: %s\n", (digitalPinToInterrupt(36) != NOT_AN_INTERRUPT) ? "YES" : "NO");
    printf("- Pin 39 interrupt attached: %s\n", (digitalPinToInterrupt(39) != NOT_AN_INTERRUPT) ? "YES" : "NO");

    // Check pin states
    printf("\nPin States:\n");
    printf("- Pin 36 state: %s\n", digitalRead(36) ? "HIGH" : "LOW");
    printf("- Pin 39 state: %s\n", digitalRead(39) ? "HIGH" : "LOW");

    // Check for signal activity
    printf("\nSignal Activity Test (5 seconds):\n");
    unsigned long startTime     = millis();
    unsigned long transitions36 = 0, transitions39 = 0;
    bool          lastState36 = digitalRead(36);
    bool          lastState39 = digitalRead(39);

    while (millis() - startTime < 5000)
    {
        bool currentState36 = digitalRead(36);
        bool currentState39 = digitalRead(39);

        if (currentState36 != lastState36)
        {
            transitions36++;
            lastState36 = currentState36;
        }

        if (currentState39 != lastState39)
        {
            transitions39++;
            lastState39 = currentState39;
        }

        delayMicroseconds(100);
    }

    printf("- Pin 36 transitions: %lu (%.1f Hz)\n", transitions36, (float)transitions36 / 5.0);
    printf("- Pin 39 transitions: %lu (%.1f Hz)\n", transitions39, (float)transitions39 / 5.0);

    // Expected frequency for MAE3 is ~250 Hz
    printf("- Expected frequency: ~250 Hz\n");

    if (transitions36 < 100)
    {
        printf("WARNING: Pin 36 has very low activity - check wiring!\n");
    }
    if (transitions39 < 100)
    {
        printf("WARNING: Pin 39 has very low activity - check wiring!\n");
    }

    printf("=== END DIAGNOSTICS ===\n\n");
}

// Simple validation function with detailed error reporting
bool validatePWMValues(unsigned long highPulse, unsigned long lowPulse, int pinNumber)
{
    printf("\n=== Validation for Pin %d ===\n", pinNumber);
    printf("High pulse: %lu us\n", highPulse);
    printf("Low pulse: %lu us\n", lowPulse);
    printf("Total period: %lu us\n", highPulse + lowPulse);

    // Check if data was bypassed (values are 0)
    if (highPulse == 0 && lowPulse == 0)
    {
        printf("Data was bypassed - invalid period detected\n");
        return false;
    }

    // Check each condition separately
    bool highOK   = (highPulse >= 1 && highPulse <= 10000);
    bool lowOK    = (lowPulse >= 1 && lowPulse <= 10000);
    bool totalOK  = ((highPulse + lowPulse) > 0);
    bool periodOK = ((highPulse + lowPulse) >= 2000 && (highPulse + lowPulse) <= 6000);

    printf("High pulse validation: %s (1-10000 us)\n", highOK ? "PASS" : "FAIL");
    printf("Low pulse validation: %s (1-10000 us)\n", lowOK ? "PASS" : "FAIL");
    printf("Total period validation: %s (>0 us)\n", totalOK ? "PASS" : "FAIL");
    printf("Period range validation: %s (2000-6000 us)\n", periodOK ? "PASS" : "FAIL");

    bool overall = highOK && lowOK && totalOK && periodOK;
    printf("Overall validation: %s\n", overall ? "PASS" : "FAIL");

    return overall;
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

    // Print bypass statistics
    printf("\n=== BYPASS STATISTICS ===\n");
    printf("Pin 36 - Total interrupts: %lu, Bypassed: %lu (%.1f%%)\n", totalInterrupts36, bypassedCount36,
           (totalInterrupts36 > 0) ? (float)bypassedCount36 * 100.0f / totalInterrupts36 : 0.0f);
    printf("Pin 39 - Total interrupts: %lu, Bypassed: %lu (%.1f%%)\n", totalInterrupts39, bypassedCount39,
           (totalInterrupts39 > 0) ? (float)bypassedCount39 * 100.0f / totalInterrupts39 : 0.0f);

    // Debug: Print raw values before validation
    printf("\n=== DEBUG: Raw PWM Values ===\n");
    printf("Pin 36 - High: %lu us, Low: %lu us, Total: %lu us\n", pulseHigh36, pulseLow36, pulseHigh36 + pulseLow36);
    printf("Pin 39 - High: %lu us, Low: %lu us, Total: %lu us\n", pulseHigh39, pulseLow39, pulseHigh39 + pulseLow39);

    // Use detailed validation
    bool valid36 = validatePWMValues(pulseHigh36, pulseLow36, 36);
    bool valid39 = validatePWMValues(pulseHigh39, pulseLow39, 39);

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

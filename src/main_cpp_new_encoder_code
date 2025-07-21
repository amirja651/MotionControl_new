#include "Pins.h"
#include <Arduino.h>

bool process36 = false, process39 = false;

struct validatePwmResult
{
    unsigned long totalPeriod;
    int           position;
    float         degrees;
    int           delta;
    bool          highOK;
    bool          lowOK;
    bool          totalOK;
    bool          periodOK;
    bool          overall;
};

// Interrupt-based PWM reading for maximum accuracy
volatile unsigned long lastRisingEdgeTime36 = 0, lastFallingEdgeTime36 = 0;
volatile unsigned long lastRisingEdgeTime39 = 0, lastFallingEdgeTime39 = 0;
volatile unsigned long pulseHigh36 = 0, pulseLow36 = 0;
volatile unsigned long pulseHigh39 = 0, pulseLow39 = 0;
volatile bool          newData36 = false, newData39 = false;

// Statistics tracking for bypassed values
volatile unsigned long bypassedCount36 = 0, bypassedCount39 = 0;
volatile unsigned long totalInterrupts36 = 0, totalInterrupts39 = 0;

// Pulse edge transition detection variables
volatile int           pulseTransitionCounter36 = 0, pulseTransitionCounter39 = 0;
volatile unsigned long lastPulseHigh36 = 0, lastPulseHigh39 = 0;

void IRAM_ATTR handleInterrupt36();
void IRAM_ATTR handleInterrupt39();

void              InitPins();
validatePwmResult validatePWMValues(unsigned long highPulse, unsigned long lowPulse, int pinNumber);
void              readPWMEncodersInterrupt();
void              diagnoseEncoderSignals();
void              resetValidatePWM(validatePwmResult* result);
unsigned long     getMedian(unsigned long arr[], int size);
float             calculateStandardDeviation(unsigned long arr[], int size);
void              readPWMEncoders();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);
    delay(1000);
    while (!Serial)
        delay(10);

    // Initialize pins
    InitPins();

    Serial.print(F("\r\n"));
    Serial.flush();
}

void loop()
{
    // Run diagnostics first (only once)
    static bool diagnosticsRun = false;
    if (!diagnosticsRun)
    {
        diagnoseEncoderSignals();
        diagnosticsRun = true;
    }

    // Interrupt-based (maximum accuracy)
    readPWMEncodersInterrupt();

    vTaskDelay(370);
}

void InitPins()
{
    // Configure pins 36 and 39 for digital PWM reading
    pinMode(36, INPUT);  // GPIO36 for PWM encoder 1
    pinMode(39, INPUT);  // GPIO39 for PWM encoder 2

    // Attach interrupts for maximum accuracy
    attachInterrupt(digitalPinToInterrupt(36), handleInterrupt36, CHANGE);
    attachInterrupt(digitalPinToInterrupt(39), handleInterrupt39, CHANGE);

    Serial.print(F("[Info][Setup] Digital pins 36 and 39 initialized with interrupts for PWM encoder reading\r\n"));

    // For disable all drivers pins - for avoid conflict in SPI bus
    // Initialize CS pins and turn them off
    for (uint8_t index = 0; index < 4; index++)
    {
        pinMode(DriverPins::CS[index], OUTPUT);
        gpio_set_level((gpio_num_t)DriverPins::CS[index], HIGH);
    }
}

// Interrupt handlers for precise timing
void IRAM_ATTR handleInterrupt36()
{
    if (process36)
        return;

    unsigned long currentTime = micros();
    totalInterrupts36++;

    if (digitalRead(36) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime36 = currentTime;
        if (lastFallingEdgeTime36 != 0)
            pulseLow36 = currentTime - lastFallingEdgeTime36;
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime36 = currentTime;
        if (lastRisingEdgeTime36 != 0)
        {
            pulseHigh36 = currentTime - lastRisingEdgeTime36;

            // Check total period before accepting the values
            // unsigned long totalPeriod = pulseHigh36 + pulseLow36;

            // MAE3 standard: period should be around 4000 us (244 Hz)
            // Accept range: 3731-4545 us (220-268 Hz)
            /*if (totalPeriod >= 3731 && totalPeriod <= 4545)
            {*/
            newData36 = true;  // Accept valid data

            // Detect pulse edge transitions
            // Count digits for current and last pulse values
            /* int currentCountDigits = countDigits(pulseHigh36);
             int lastCountDigits    = countDigits(lastPulseValue36);

             // Detect transition from 1-2 digits to 4 digits (increasing)
             if (pulseHigh36 > 4000 && (lastCountDigits == 1 || lastCountDigits == 2))
                 pulseTransitionCounter36++;

             // Detect transition from 4 digits to 1-2 digits (decreasing)
             else if (lastPulseValue36 > 4000 && (currentCountDigits == 1 || currentCountDigits == 2))
                 pulseTransitionCounter36--;

             lastPulseValue36 = pulseHigh36;*/
            /*}
            else
            {
                // Bypass invalid data - reset values
                pulseHigh36 = 0;
                pulseLow36  = 0;
                newData36   = false;
                bypassedCount36++;
            }*/
        }
    }
}

void IRAM_ATTR handleInterrupt39()
{
    if (process39)
        return;

    unsigned long currentTime = micros();
    totalInterrupts39++;

    if (digitalRead(39) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime39 = currentTime;
        if (lastFallingEdgeTime39 != 0)
            pulseLow39 = currentTime - lastFallingEdgeTime39;
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime39 = currentTime;
        if (lastRisingEdgeTime39 != 0)
        {
            pulseHigh39 = currentTime - lastRisingEdgeTime39;

            // Check total period before accepting the values
            // unsigned long totalPeriod = pulseHigh39 + pulseLow39;

            // MAE3 standard: period should be around 4000 us (244 Hz)
            // Accept range: 3731-4545 us (220-268 Hz)
            /*if (totalPeriod >= 3731 && totalPeriod <= 4545)
            {*/
            newData39 = true;  // Accept valid data

            // Detect pulse edge transitions
            // Count digits for current and last pulse values
            /*int currentCountDigits = countDigits(pulseHigh39);
            int lastCountDigits    = countDigits(lastPulseValue39);

            // Detect transition from 1-2 digits to 4 digits (increasing)
            if (pulseHigh39 > 4000 && (lastCountDigits == 1 || lastCountDigits == 2))
                pulseTransitionCounter39++;

            // Detect transition from 4 digits to 1-2 digits (decreasing)
            else if (lastPulseValue39 > 4000 && (currentCountDigits == 1 || currentCountDigits == 2))
                pulseTransitionCounter39--;

            lastPulseValue39 = pulseHigh39;*/
            /* }
             else
             {
                 // Bypass invalid data - reset values
                 pulseHigh39 = 0;
                 pulseLow39  = 0;
                 newData39   = false;
                 bypassedCount39++;
             }*/
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

    printf("- Pin 36 transitions: %lu (%.1f Hz)\n", transitions36, (float)transitions36 / 10.0);
    printf("- Pin 39 transitions: %lu (%.1f Hz)\n", transitions39, (float)transitions39 / 10.0);

    // Expected frequency for MAE3 is ~250 Hz
    printf("- Expected frequency: MIN. 220 Hz, MAX. 268 Hz, TYP. 244 Hz\n");

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

void resetValidatePWM(validatePwmResult* result)
{
    result->totalPeriod = 0;
    result->position    = 0;
    result->degrees     = 0.0f;
    result->highOK      = false;
    result->lowOK       = false;
    result->totalOK     = false;
    result->periodOK    = false;
    result->overall     = false;
}

// Simple validation function with detailed error reporting
validatePwmResult validatePWMValues(unsigned long highPulse, unsigned long lowPulse, int pinNumber)
{
    validatePwmResult result;
    resetValidatePWM(&result);

    result.totalPeriod = highPulse + lowPulse;

    printf("\n=== Validation for Pin %d ===\n", pinNumber);
    printf("High pulse: %lu us\n", highPulse);
    printf("Low pulse: %lu us\n", lowPulse);
    printf("Total period: %lu us\n", result.totalPeriod);

    // Check if data was bypassed (values are 0)
    if (highPulse == 0 && lowPulse == 0)
    {
        printf("Data was bypassed - invalid period detected\n");

        return result;
    }

    // Check each condition separately
    result.highOK   = (highPulse >= 1 && highPulse <= 4302);  // 4097
    result.lowOK    = (lowPulse >= 1 && lowPulse <= 4302);    // 4097//amir
    result.totalOK  = ((result.totalPeriod) > 0);
    result.periodOK = ((result.totalPeriod) >= 3731 && (result.totalPeriod) <= 4545);
    result.overall  = result.highOK && result.lowOK && result.totalOK && result.periodOK;

    printf("High pulse validation: %s (1-4097 us)\n", result.highOK ? "PASS" : "FAIL");
    printf("Low pulse validation: %s (1-4097 us)\n", result.lowOK ? "PASS" : "FAIL");
    printf("Total period validation: %s (>0 us)\n", result.totalOK ? "PASS" : "FAIL");
    printf("Period range validation: %s (3731-4545 us)\n", result.periodOK ? "PASS" : "FAIL");
    printf("Overall validation: %s\n", result.overall ? "PASS" : "FAIL");

    return result;
}

bool enteredForwardWrapZone36 = false;
bool enteredReverseWrapZone36 = false;
bool enteredForwardWrapZone39 = false;
bool enteredReverseWrapZone39 = false;

// Interrupt-based PWM reading method
void readPWMEncodersInterrupt()
{
    process36 = true;
    process39 = true;

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
    printf("Pin 36 - Total interrupts: %lu, Bypassed: %lu (%.1f%%)\n", totalInterrupts36, bypassedCount36, (totalInterrupts36 > 0) ? (float)bypassedCount36 * 100.0f / totalInterrupts36 : 0.0f);
    printf("Pin 39 - Total interrupts: %lu, Bypassed: %lu (%.1f%%)\n", totalInterrupts39, bypassedCount39, (totalInterrupts39 > 0) ? (float)bypassedCount39 * 100.0f / totalInterrupts39 : 0.0f);

    // Print raw values before validation
    printf("\n===  Raw PWM Values ===\n");
    printf("Pin 36 - High: %lu us, Low: %lu us, Total: %lu us\n", pulseHigh36, pulseLow36, pulseHigh36 + pulseLow36);
    printf("Pin 39 - High: %lu us, Low: %lu us, Total: %lu us\n", pulseHigh39, pulseLow39, pulseHigh39 + pulseLow39);

    // Use detailed validation
    validatePwmResult valid36 = validatePWMValues(pulseHigh36, pulseLow36, 36);
    validatePwmResult valid39 = validatePWMValues(pulseHigh39, pulseLow39, 39);

    // Calculate position using MAE3 PWM formula for 12-bit
    if (valid36.overall)
    {
        int position36   = ((pulseHigh36 * 4098) / valid36.totalPeriod) - 1;
        valid36.position = constrain(position36, 0, 4095);
        valid36.degrees  = (valid36.position * 360.0f) / 4096.0f;
        valid36.delta    = pulseHigh36 - lastPulseHigh36;

        if (lastPulseHigh36 > 3000 && pulseHigh36 < 200 && !enteredForwardWrapZone36)
        {
            pulseTransitionCounter36++;
            enteredForwardWrapZone36 = true;
            enteredReverseWrapZone36 = false;  // reset other direction
        }
        else if (lastPulseHigh36 < 200 && pulseHigh36 > 3000 && !enteredReverseWrapZone36)
        {
            pulseTransitionCounter36--;
            enteredForwardWrapZone36 = false;
            enteredReverseWrapZone36 = true;  // reset other direction
        }

        // Reset lock flags when encoder is in mid-range (normal zone)
        else if (pulseHigh36 > 200 + 200 && pulseHigh36 < 3000 - 200)
        {
            enteredForwardWrapZone36 = false;
            enteredReverseWrapZone36 = false;
        }

        lastPulseHigh36 = pulseHigh36;
    }

    if (valid39.overall)
    {
        int position39   = ((pulseHigh39 * 4098) / valid39.totalPeriod) - 1;
        valid39.position = constrain(position39, 0, 4095);
        valid39.degrees  = (valid39.position * 360.0f) / 4096.0f;
        valid39.delta    = pulseHigh39 - lastPulseHigh39;

        if (lastPulseHigh39 > 3000 && pulseHigh39 < 200 && !enteredForwardWrapZone39)
        {
            pulseTransitionCounter39++;
            enteredForwardWrapZone39 = true;
            enteredReverseWrapZone39 = false;  // reset other direction
        }
        else if (lastPulseHigh39 < 200 && pulseHigh39 > 3000 && !enteredReverseWrapZone39)
        {
            pulseTransitionCounter39--;
            enteredForwardWrapZone39 = false;
            enteredReverseWrapZone39 = true;  // reset other direction
        }

        // Reset flag when position returns to mid-range
        if (pulseHigh39 > 300 && pulseHigh39 < 3000)
        {
            enteredForwardWrapZone39 = false;
            enteredReverseWrapZone39 = false;
        }

        // Reset lock flags when encoder is in mid-range (normal zone)
        else if (pulseHigh39 > 200 + 200 && pulseHigh39 < 3000 - 200)
        {
            enteredForwardWrapZone39 = false;
            enteredReverseWrapZone39 = false;
        }

        lastPulseHigh39 = pulseHigh39;
    }

    // Check if position has changed
    static int lastPrintPosition36 = -1, lastPrintPosition39 = -1;

    if (abs(valid36.position - lastPrintPosition36) > 1 || abs(valid39.position - lastPrintPosition39) > 1)
    {
        lastPrintPosition36 = valid36.position;
        lastPrintPosition39 = valid39.position;

        // Print results
        Serial.println();
        printf("┌────────────────────────────────────────────────────────────────────────────────────────────────────────────┐\n");
        printf("│                                     Interrupt-Based PWM Readings                                           │\n");
        printf("├────────────────────────────────────────────────────────────────────────────────────────────────────────────┤\n");
        printf("│ Pin │ Position │ Degrees  │ High Pulse │ Low Pulse │ Total Period │ Status │ Transition │ Bypassed │ Delta |\n");
        printf("├─────┼──────────┼──────────┼────────────┼───────────┼──────────────┼────────┼────────────┼──────────────────┤\n");
        printf("│ 36  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %s │ %10d │ %8lu │ %5d │\n", valid36.position, valid36.degrees, pulseHigh36, pulseLow36, valid36.totalPeriod, valid36.overall ? "OK    " : "ERR   ", pulseTransitionCounter36, bypassedCount36, valid36.delta);
        printf("│ 39  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %s │ %10d │ %8lu │ %5d │\n", valid39.position, valid39.degrees, pulseHigh39, pulseLow39, valid39.totalPeriod, valid39.overall ? "OK    " : "ERR   ", pulseTransitionCounter39, bypassedCount39, valid39.delta);
        printf("└────────────────────────────────────────────────────────────────────────────────────────────────────────────┘\n");
    }

    process36 = false;
    process39 = false;
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
    bool valid36 = (medianHigh36 >= MIN_PULSE_US && medianHigh36 <= MAX_PULSE_US && medianLow36 >= MIN_PULSE_US && medianLow36 <= MAX_PULSE_US && medianPeriod36 > 0 && medianPeriod36 <= 4098);

    bool valid39 = (medianHigh39 >= MIN_PULSE_US && medianHigh39 <= MAX_PULSE_US && medianLow39 >= MIN_PULSE_US && medianLow39 <= MAX_PULSE_US && medianPeriod39 > 0 && medianPeriod39 <= 4098);

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
    printf("│ 36  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %6.1f │ %s │\n", position36, degrees36, medianHigh36, medianLow36, medianPeriod36, stdDev36, valid36 ? "OK    " : "ERR   ");
    printf("│ 39  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %6.1f │ %s │\n", position39, degrees39, medianHigh39, medianLow39, medianPeriod39, stdDev39, valid39 ? "OK    " : "ERR   ");
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

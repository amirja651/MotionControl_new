#include "Pins.h"
#include <Arduino.h>

// New method for reading voltage from encoder pins
void readEncoderVoltages();

// New method for reading PWM encoder values
void readPWMEncoders();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);

    delay(1000);
    while (!Serial)
        delay(10);

    // Initialize analog pins for voltage reading
    analogReadResolution(12);        // Set ADC resolution to 12 bits (0-4095)
    analogSetAttenuation(ADC_11db);  // Set attenuation for 0-3.3V range

    // Configure pins 34 and 35 for analog reading
    // Note: These pins are already configured as analog inputs by default on ESP32
    // GPIO34 (VP) and GPIO35 (VN) are dedicated analog input pins
    Serial.print(F("[Info][Setup] Analog pins 34 and 35 initialized for voltage reading\r\n"));

    // Configure pins 36 and 39 for analog reading
    // Note: These pins are already configured as analog inputs by default on ESP32
    // GPIO36 (VP) and GPIO39 (VN) are dedicated analog input pins
    Serial.print(F("[Info][Setup] Analog pins 36 and 39 initialized for voltage reading\r\n"));

    // Configure pins 36 and 39 for digital PWM reading
    pinMode(36, INPUT);  // GPIO36 for PWM encoder 1
    pinMode(39, INPUT);  // GPIO39 for PWM encoder 2
    Serial.print(F("[Info][Setup] Digital pins 36 and 39 initialized for PWM encoder reading\r\n"));

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
    readPWMEncoders();

    delay(1000);
}

// New method for reading voltage from encoder pins
void readEncoderVoltages()
{
    // Read analog values from pins 34 and 35
    static int prevVoltagePin34 = -1;
    static int prevVoltagePin35 = -1;

    int voltagePin34 = analogRead(34);  // GPIO34 (VP)
    int voltagePin35 = analogRead(35);  // GPIO35 (VN)

    // اگر مقادیر 4 پین نسبت به مقادیر قبلیشون تغییر نداشتند خارج شو
    if ((voltagePin34 == prevVoltagePin34 && voltagePin35 == prevVoltagePin35) || (voltagePin34 == 0 && voltagePin35 == 0))
    {
        return;
    }

    // Update previous values
    prevVoltagePin34 = voltagePin34;
    prevVoltagePin35 = voltagePin35;

    // Convert to voltage (ESP32 ADC reference is 3.3V, 12-bit resolution = 4095)
    float voltage34 = (voltagePin34 * 3.3f) / 4095.0f;
    float voltage35 = (voltagePin35 * 3.3f) / 4095.0f;

    // Print the voltage values
    char buffer[256];

    // Print table header
    sprintf(buffer, "┌─────────┬─────────┬──────────┐\r\n");
    Serial.print(buffer);
    sprintf(buffer, "│ Pin     │ Voltage │ ADC      │\r\n");
    Serial.print(buffer);
    sprintf(buffer, "├─────────┼─────────┼──────────┤\r\n");
    Serial.print(buffer);

    // Print data rows
    sprintf(buffer, "│ Pin 34  │ %6.3fV │ %6d   │\r\n", voltage34, voltagePin34);
    Serial.print(buffer);
    sprintf(buffer, "│ Pin 35  │ %6.3fV │ %6d   │\r\n", voltage35, voltagePin35);
    Serial.print(buffer);

    // Print table footer
    sprintf(buffer, "└─────────┴─────────┴──────────┘\r\n");
    Serial.print(buffer);
}

// New method for reading PWM encoder values
void readPWMEncoders()
{
    // Variables for PWM measurement
    unsigned long pulseHigh36 = 0, pulseLow36 = 0;
    unsigned long pulseHigh39 = 0, pulseLow39 = 0;
    unsigned long totalPeriod36 = 0, totalPeriod39 = 0;

    // Measure PWM for pin 36
    unsigned long startTime = micros();
    while (digitalRead(36) == HIGH && (micros() - startTime) < 5000)
    {
        // Wait for falling edge
    }
    startTime = micros();
    while (digitalRead(36) == LOW && (micros() - startTime) < 5000)
    {
        // Wait for rising edge
    }

    // Measure high pulse width
    startTime = micros();
    while (digitalRead(36) == HIGH && (micros() - startTime) < 5000)
    {
        pulseHigh36 = micros() - startTime;
    }

    // Measure low pulse width
    startTime = micros();
    while (digitalRead(36) == LOW && (micros() - startTime) < 5000)
    {
        pulseLow36 = micros() - startTime;
    }

    totalPeriod36 = pulseHigh36 + pulseLow36;

    // Measure PWM for pin 39
    startTime = micros();
    while (digitalRead(39) == HIGH && (micros() - startTime) < 5000)
    {
        // Wait for falling edge
    }
    startTime = micros();
    while (digitalRead(39) == LOW && (micros() - startTime) < 5000)
    {
        // Wait for rising edge
    }

    // Measure high pulse width
    startTime = micros();
    while (digitalRead(39) == HIGH && (micros() - startTime) < 5000)
    {
        pulseHigh39 = micros() - startTime;
    }

    // Measure low pulse width
    startTime = micros();
    while (digitalRead(39) == LOW && (micros() - startTime) < 5000)
    {
        pulseLow39 = micros() - startTime;
    }

    totalPeriod39 = pulseHigh39 + pulseLow39;

    // Calculate position using MAE3 PWM formula for 12-bit
    // x = ((t_on * 4098) / (t_on + t_off)) - 1
    int position36 = 0, position39 = 0;

    if (totalPeriod36 > 0)
    {
        position36 = ((pulseHigh36 * 4098) / totalPeriod36) - 1;
        if (position36 > 4094)
            position36 = 4095;
        if (position36 < 0)
            position36 = 0;
    }

    if (totalPeriod39 > 0)
    {
        position39 = ((pulseHigh39 * 4098) / totalPeriod39) - 1;
        if (position39 > 4094)
            position39 = 4095;
        if (position39 < 0)
            position39 = 0;
    }

    // Convert to degrees (360° / 4096 positions)
    float degrees36 = (position36 * 360.0f) / 4096.0f;
    float degrees39 = (position39 * 360.0f) / 4096.0f;

    // Print results
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│                                  PWM Encoder Readings                      │\n");
    printf("├────────────────────────────────────────────────────────────────────────────┤\n");
    printf("│ Pin │ Position │ Degrees  │ High Pulse │ Low Pulse │ Total Period │ Status │\n");
    printf("├─────┼──────────┼──────────┼────────────┼───────────┼──────────────┼────────┤\n");
    printf("│ 36  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %s │\n", position36, degrees36, pulseHigh36, pulseLow36, totalPeriod36,
           (totalPeriod36 > 0) ? "OK    " : "ERR   ");
    printf("│ 39  │ %8d │ %7.2f° │ %10lu │ %9lu │ %12lu │ %s │\n", position39, degrees39, pulseHigh39, pulseLow39, totalPeriod39,
           (totalPeriod39 > 0) ? "OK    " : "ERR   ");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n");
}

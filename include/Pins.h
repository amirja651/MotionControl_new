#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

struct DriverPins
{
    static const uint16_t DIR[4];
    static const uint16_t STEP[4];
    static const uint16_t EN[4];
    static const uint16_t CS[4];
};

struct SPIPins
{
    static const uint16_t MOSI = 23;
    static const uint16_t MISO = 19;
    static const uint16_t SCK  = 18;
};

struct EncoderPins
{
    static const uint16_t SIGNAL[4];
};

struct MultiplexerPins
{                                    //{22, 4, 32, 27}
    static const uint16_t S0  = 22;  // Select line 0
    static const uint16_t S1  = 4;   // Select line 1
    static const uint16_t DIR = 32;  // Direction signal (connected to common Z)
};

struct VoltageMonitorPins
{
    static const uint16_t POWER_3_3 = 27;
};

#endif  // DRIVER_PINS_H
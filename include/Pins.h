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

#endif  // DRIVER_PINS_H
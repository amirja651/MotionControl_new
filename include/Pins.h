#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

#define NUM_DRIVERS 2

struct DriverPins
{
    static const uint16_t DIR[NUM_DRIVERS];
    static const uint16_t STEP[NUM_DRIVERS];
    static const uint16_t EN[NUM_DRIVERS];
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
    static const uint16_t SIGNAL[NUM_DRIVERS];
};

#endif  // DRIVER_PINS_H
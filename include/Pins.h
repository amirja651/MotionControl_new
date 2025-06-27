#ifndef PINS_H
#define PINS_H

#include "Defines.h"
#include <Arduino.h>

struct DriverPins
{
    static const uint16_t DIR[NUM_MOTORS];
    static const uint16_t STEP[NUM_MOTORS];
    static const uint16_t EN[NUM_MOTORS];
    static const uint16_t CS[NUM_DRIVERS];
};

struct SPIPins
{
    static const uint16_t MOSI = 23;
    static const uint16_t MISO = 19;
    static const uint16_t SCK  = 18;
};

struct EncoderPins
{
    static const uint16_t SIGNAL[NUM_ENCODERS];
};

#endif  // DRIVER_PINS_H
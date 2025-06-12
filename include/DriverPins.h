#ifndef DRIVER_PINS_H
#define DRIVER_PINS_H

#include <Arduino.h>

static constexpr uint8_t NUM_DRIVERS = 4;

struct DriverPins
{
    static const uint16_t DIR[NUM_DRIVERS];
    static const uint16_t STEP[NUM_DRIVERS];
    static const uint16_t EN[NUM_DRIVERS];
    static const uint16_t CS[NUM_DRIVERS];
};

#endif  // DRIVER_PINS_H
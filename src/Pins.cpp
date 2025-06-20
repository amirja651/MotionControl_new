#include "Pins.h"

#if NUM_DRIVERS == 1
const uint16_t DriverPins::DIR[NUM_DRIVERS]     = {22};
const uint16_t DriverPins::STEP[NUM_DRIVERS]    = {21};
const uint16_t DriverPins::EN[NUM_DRIVERS]      = {17};
const uint16_t EncoderPins::SIGNAL[NUM_DRIVERS] = {36};

#elif NUM_DRIVERS == 2
const uint16_t DriverPins::DIR[NUM_DRIVERS]     = {22, 4};
const uint16_t DriverPins::STEP[NUM_DRIVERS]    = {21, 16};
const uint16_t DriverPins::EN[NUM_DRIVERS]      = {17, 15};
const uint16_t EncoderPins::SIGNAL[NUM_DRIVERS] = {36, 39};

#elif NUM_DRIVERS == 3
const uint16_t DriverPins::DIR[NUM_DRIVERS]     = {22, 4, 32};
const uint16_t DriverPins::STEP[NUM_DRIVERS]    = {21, 16, 33};
const uint16_t DriverPins::EN[NUM_DRIVERS]      = {17, 15, 26};
const uint16_t EncoderPins::SIGNAL[NUM_DRIVERS] = {36, 39, 34};

#elif NUM_DRIVERS == 4
const uint16_t DriverPins::DIR[NUM_DRIVERS]     = {22, 4, 32, 27};
const uint16_t DriverPins::STEP[NUM_DRIVERS]    = {21, 16, 33, 14};
const uint16_t DriverPins::EN[NUM_DRIVERS]      = {17, 15, 26, 13};
const uint16_t EncoderPins::SIGNAL[NUM_DRIVERS] = {36, 39, 34, 35};

#endif

// for disable all drivers pins - for avoid conflict in SPI bus
const uint16_t DriverPins::CS[4] = {5, 2, 25, 12};
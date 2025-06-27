#include "Pins.h"
#include "Defines.h"

#if NUM_MOTORS == 1
const uint16_t DriverPins::DIR[NUM_MOTORS]  = {22};
const uint16_t DriverPins::STEP[NUM_MOTORS] = {21};
const uint16_t DriverPins::EN[NUM_MOTORS]   = {17};
#elif NUM_MOTORS == 2
const uint16_t DriverPins::DIR[NUM_MOTORS]  = {22, 4};
const uint16_t DriverPins::STEP[NUM_MOTORS] = {21, 16};
const uint16_t DriverPins::EN[NUM_MOTORS]   = {17, 15};
#elif NUM_MOTORS == 3
const uint16_t DriverPins::DIR[NUM_MOTORS]  = {22, 4, 32};
const uint16_t DriverPins::STEP[NUM_MOTORS] = {21, 16, 33};
const uint16_t DriverPins::EN[NUM_MOTORS]   = {17, 15, 26};
#elif NUM_MOTORS == 4
const uint16_t DriverPins::DIR[NUM_MOTORS]  = {22, 4, 32, 27};
const uint16_t DriverPins::STEP[NUM_MOTORS] = {21, 16, 33, 14};
const uint16_t DriverPins::EN[NUM_MOTORS]   = {17, 15, 26, 13};
#endif

#if NUM_ENCODERS == 1
const uint16_t EncoderPins::SIGNAL[NUM_ENCODERS] = {36};
#elif NUM_ENCODERS == 2
const uint16_t EncoderPins::SIGNAL[NUM_ENCODERS] = {36, 39};
#elif NUM_ENCODERS == 3
const uint16_t EncoderPins::SIGNAL[NUM_ENCODERS] = {36, 39, 34};
#elif NUM_ENCODERS == 4
const uint16_t EncoderPins::SIGNAL[NUM_ENCODERS] = {36, 39, 34, 35};
#endif

// for disable all drivers pins - for avoid conflict in SPI bus
#if NUM_DRIVERS == 1
const uint16_t DriverPins::CS[NUM_DRIVERS] = {5};
#elif NUM_DRIVERS == 2
const uint16_t DriverPins::CS[NUM_DRIVERS] = {5, 2};
#elif NUM_DRIVERS == 3
const uint16_t DriverPins::CS[NUM_DRIVERS] = {5, 2, 25};
#elif NUM_DRIVERS == 4
const uint16_t DriverPins::CS[NUM_DRIVERS] = {5, 2, 25, 12};
#endif
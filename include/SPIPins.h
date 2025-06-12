#ifndef SPI_H
#define SPI_H

#include <Arduino.h>

struct SPIPins
{
    static const uint16_t MOSI = 23;
    static const uint16_t MISO = 19;
    static const uint16_t SCK  = 18;
};

#endif  // SPI_H
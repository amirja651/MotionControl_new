// File: MiniLinkMaster.h
#pragma once
#include "SpiProto.h"
#include <Arduino.h>
#include <SPI.h>

#ifndef LINK_SCLK
    #define LINK_SCLK 18
    #define LINK_MISO 19
    #define LINK_MOSI 23
    #define LINK_CS   5
#endif

class MiniLinkMaster
{
public:
    void begin(uint32_t hz = 8'000'000)
    {
        _spi.begin(LINK_SCLK, LINK_MISO, LINK_MOSI, LINK_CS);
        pinMode(LINK_CS, OUTPUT);
        digitalWrite(LINK_CS, HIGH);
        _hz = hz;
    }

    // 16-byte full-duplex transaction
    bool transact(const Link::Packet& tx, Link::Packet& rx)
    {
        digitalWrite(LINK_CS, LOW);
        _spi.beginTransaction(SPISettings(_hz, MSBFIRST, SPI_MODE0));
        _spi.transferBytes((uint8_t*)&tx, (uint8_t*)&rx, Link::PACK_SIZE);
        _spi.endTransaction();
        digitalWrite(LINK_CS, HIGH);
        return Link::checkPacket(rx) && (rx.seq == tx.seq);
    }

    // Convenient examples
    bool ping()
    {
        Link::Packet tx{}, rx{};
        Link::makePacket(tx, nextSeq(), Link::PING, nullptr, 0);
        if (!transact(tx, rx))
            return false;
        return (rx.cmd == Link::PONG);
    }

    bool getPosition(uint8_t motor, int32_t& steps_out)
    {
        Link::Packet tx{}, rx{};
        uint8_t      pl[1] = {motor};
        Link::makePacket(tx, nextSeq(), Link::GET_POS, pl, 1);
        if (!transact(tx, rx))
            return false;
        if (rx.cmd != Link::POS)
            return false;
        steps_out = (int32_t)((uint32_t)rx.payload[1] | ((uint32_t)rx.payload[2] << 8) | ((uint32_t)rx.payload[3] << 16) | ((uint32_t)rx.payload[4] << 24));
        return true;
    }

    bool moveAbs(uint8_t motor, int32_t target, uint16_t vmax, uint16_t amax)
    {
        Link::Packet tx{}, rx{};
        uint8_t      pl[Link::PAYLOAD_SZ] = {0};
        pl[0]                             = motor;
        pl[1]                             = 0;  // axis=0 (e.g. map rotary/linear later)
        // target (LE)
        pl[2] = (uint8_t)(target);
        pl[3] = (uint8_t)(target >> 8);
        pl[4] = (uint8_t)(target >> 16);
        pl[5] = (uint8_t)(target >> 24);
        // vmax
        pl[6] = (uint8_t)vmax;
        pl[7] = (uint8_t)(vmax >> 8);
        // amax
        pl[8] = (uint8_t)amax;
        pl[9] = (uint8_t)(amax >> 8);
        Link::makePacket(tx, nextSeq(), Link::MOVE_ABS, pl, 10);
        if (!transact(tx, rx))
            return false;
        return (rx.cmd == Link::ACK);
    }

private:
    SPIClass _spi{VSPI};
    uint32_t _hz  = 8'000'000;
    uint8_t  _seq = 0;
    uint8_t  nextSeq()
    {
        return ++_seq;
    }
};

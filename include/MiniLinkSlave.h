// File: MiniLinkSlave.h
#pragma once
#include "SpiProto.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include <Arduino.h>

#ifndef LINK_SCLK
    #define LINK_SCLK 7  // Example for S3 Mini: Change
    #define LINK_MISO 8
    #define LINK_MOSI 13
    #define LINK_CS   10
#endif

class MiniLinkSlave
{
public:
    void begin2()
    {
        // Pins
        gpio_set_pull_mode((gpio_num_t)LINK_MISO, GPIO_FLOATING);
        gpio_set_pull_mode((gpio_num_t)LINK_MOSI, GPIO_FLOATING);
        gpio_set_pull_mode((gpio_num_t)LINK_SCLK, GPIO_FLOATING);
        gpio_set_pull_mode((gpio_num_t)LINK_CS, GPIO_PULLUP_ONLY);

        spi_bus_config_t bus{};
        bus.mosi_io_num     = LINK_MOSI;
        bus.miso_io_num     = LINK_MISO;
        bus.sclk_io_num     = LINK_SCLK;
        bus.quadwp_io_num   = -1;
        bus.quadhd_io_num   = -1;
        bus.max_transfer_sz = Link::PACK_SIZE;

        spi_slave_interface_config_t slv{};
        slv.mode         = 0;
        slv.spics_io_num = LINK_CS;
        slv.queue_size   = 2;
        slv.flags        = 0;

        ESP_ERROR_CHECK(spi_slave_initialize(SPI3_HOST, &bus, &slv, SPI_DMA_CH_AUTO));
    }

    // Service loop; one 16-byte frame at a time
    void serviceOnce()
    {
        alignas(4) Link::Packet rx{}, tx{};
        // Default response NACK
        makeNack(tx, 0xFE);

        spi_slave_transaction_t t{};
        t.length    = Link::PACK_SIZE * 8;
        t.tx_buffer = &tx;
        t.rx_buffer = &rx;

        // Waiting for the master transaction
        esp_err_t r = spi_slave_transmit(SPI3_HOST, &t, portMAX_DELAY);
        if (r != ESP_OK)
            return;

        if (!Link::checkPacket(rx))
        {
            // bad CRC → NACK with received seq
            tx.seq = rx.seq;
            makeNack(tx, 0x01);
            return;
        }

        // Fill in the appropriate answer
        handleCommand(rx, tx);
    }

    // Connect these two functions to the internal motor/encoder code:
    int32_t getMotorSteps(uint8_t motor)
    {
        // TODO: actual value from PositionController
        (void)motor;
        return _dbg_pos;  // Sample
    }
    uint16_t getEncoderRaw(uint8_t motor)
    {
        // TODO: actual value from MAE3
        (void)motor;
        return _dbg_enc;  // Sample
    }
    void moveAbs(uint8_t motor, int32_t tgt, uint16_t vmax, uint16_t amax)
    {
        (void)motor;
        (void)vmax;
        (void)amax;
        _dbg_pos = tgt;  // Example; replace with actual move queue
    }
    void stop(uint8_t motor)
    {
        (void)motor; /* TODO */
    }
    void enable(uint8_t motor, bool on)
    {
        (void)motor;
        (void)on; /* TODO */
    }

private:
    int32_t  _dbg_pos = 0;
    uint16_t _dbg_enc = 0;

    static inline void makeAck(Link::Packet& p, uint8_t seq, uint8_t st = 0)
    {
        uint8_t pl[Link::PAYLOAD_SZ] = {st};
        Link::makePacket(p, seq, Link::ACK, pl, 1);
    }
    static inline void makeNack(Link::Packet& p, uint8_t err)
    {
        uint8_t pl[Link::PAYLOAD_SZ] = {err};
        Link::makePacket(p, p.seq, Link::NACK, pl, 1);
    }

    void handleCommand(const Link::Packet& rx, Link::Packet& tx)
    {
        switch (rx.cmd)
        {
            case Link::PING:
            {
                uint8_t pl[1] = {(uint8_t)(millis() & 0xFF)};
                Link::makePacket(tx, rx.seq, Link::PONG, pl, 1);
            }
            break;

            case Link::GET_POS:
            {
                uint8_t m     = rx.payload[0];
                int32_t s     = getMotorSteps(m);
                uint8_t pl[6] = {m, (uint8_t)(s), (uint8_t)(s >> 8), (uint8_t)(s >> 16), (uint8_t)(s >> 24)};
                Link::makePacket(tx, rx.seq, Link::POS, pl, 5);
            }
            break;

            case Link::GET_ENC:
            {
                uint8_t  m     = rx.payload[0];
                uint16_t raw   = getEncoderRaw(m);
                uint8_t  pl[3] = {m, (uint8_t)raw, (uint8_t)(raw >> 8)};
                Link::makePacket(tx, rx.seq, Link::ENC, pl, 3);
            }
            break;

            case Link::MOVE_ABS:
            case Link::MOVE_REL:
            {
                uint8_t m = rx.payload[0];
                // axis in payload[1]; currently ignored
                int32_t tgt = (int32_t)((uint32_t)rx.payload[2] | ((uint32_t)rx.payload[3] << 8) | ((uint32_t)rx.payload[4] << 16) | ((uint32_t)rx.payload[5] << 24));
                if (rx.cmd == Link::MOVE_REL)
                    _dbg_pos += tgt;
                uint16_t vmax = (uint16_t)(rx.payload[6] | (rx.payload[7] << 8));
                uint16_t amax = (uint16_t)(rx.payload[8] | (rx.payload[9] << 8));
                moveAbs(m, tgt, vmax, amax);
                makeAck(tx, rx.seq, Link::ST_OK);
            }
            break;

            case Link::STOP:
            {
                stop(rx.payload[0]);
                makeAck(tx, rx.seq, Link::ST_OK);
            }
            break;

            case Link::ENABLE:
            {
                enable(rx.payload[0], rx.payload[1] != 0);
                makeAck(tx, rx.seq, Link::ST_OK);
            }
            break;

            default:
                makeNack(tx, 0xFF);
                tx.seq = rx.seq;
                break;
        }
    }
};

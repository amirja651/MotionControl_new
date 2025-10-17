// File: SpiProto.h
#pragma once
#include <stdint.h>

namespace Link
{

    // --- Fixed and style settings ---
    static constexpr uint8_t MAGIC      = 0xA5;
    static constexpr uint8_t PACK_SIZE  = 16;    // Fixed frame 16 bytes
    static constexpr uint8_t PAYLOAD_SZ = 10;    // 16 - (magic,seq,cmd,crc) = 12 → but 2 reservations = 10
    static constexpr uint8_t CRC_POLY   = 0x07;  // CRC8-ATM
    static constexpr uint8_t CRC_INIT   = 0x00;

    // Commands (host → mini)
    enum Cmd : uint8_t
    {
        PING      = 0x00,
        MOVE_ABS  = 0x01,  // payload: [motor(1), axis(1), int32 target, uint16 vmax, uint16 amax]
        MOVE_REL  = 0x02,  // as above
        STOP      = 0x03,  // [motor]
        ENABLE    = 0x04,  // [motor, 1=on/0=off]
        GET_POS   = 0x05,  // [motor]
        GET_ENC   = 0x06,  // [motor]
        SET_PARAM = 0x07,  // [id(1), value(4)] Optional usage
    };

    // Responses (mini → host)
    enum Rsp : uint8_t
    {
        ACK    = 0x80,  // payload: [status]
        NACK   = 0x81,  // payload: [err]
        POS    = 0x82,  // payload: [motor, int32 steps]
        ENC    = 0x83,  // payload: [motor, uint16 raw12]
        PONG   = 0x84,  // payload: [uptime_low8]
        STATUS = 0x85,  // payload: [status bitmap]
    };

    // Status/Error (style bitmap)
    enum StatusBits : uint8_t
    {
        ST_OK         = 0,
        ST_BUSY       = 1 << 0,
        ST_UVLO       = 1 << 1,
        ST_DRIVER_ERR = 1 << 2,
    };

    struct Packet
    {
        uint8_t magic;
        uint8_t seq;
        uint8_t cmd;
        uint8_t payload[PAYLOAD_SZ];  // zero-filled
        uint8_t crc;
    } __attribute__((packed));

    inline uint8_t crc8(const uint8_t* d, uint8_t len, uint8_t crc = CRC_INIT)
    {
        for (uint8_t i = 0; i < len; i++)
        {
            crc ^= d[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC_POLY) : (uint8_t)(crc << 1);
        }
        return crc;
    }

    inline void makePacket(Packet& p, uint8_t seq, uint8_t cmd, const void* pl = nullptr, uint8_t pl_len = 0)
    {
        p.magic = MAGIC;
        p.seq   = seq;
        p.cmd   = cmd;
        for (uint8_t i = 0; i < PAYLOAD_SZ; i++)
            p.payload[i] = 0;
        if (pl && pl_len)
        {
            if (pl_len > PAYLOAD_SZ)
                pl_len = PAYLOAD_SZ;
            const uint8_t* src = (const uint8_t*)pl;
            for (uint8_t i = 0; i < pl_len; i++)
                p.payload[i] = src[i];
        }
        // CRC on the first 15 bytes
        p.crc = crc8(reinterpret_cast<const uint8_t*>(&p), PACK_SIZE - 1);
    }

    inline bool checkPacket(const Packet& p)
    {
        if (p.magic != MAGIC)
            return false;
        uint8_t c = crc8(reinterpret_cast<const uint8_t*>(&p), PACK_SIZE - 1);
        return c == p.crc;
    }

}  // namespace Link

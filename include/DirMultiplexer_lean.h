// =============================
// File: DirMultiplexer.h
// Lean, allocation‑free, RT‑friendly 2→4 DIR multiplexer for ESP32 (Arduino)
// - Fast GPIO (single‑cycle W1TS/W1TC) in hot path (configurable)
// - No dynamic allocation, no blocking delays, ISR‑safe inlines in IRAM
// - Optional logging; API kept compatible with your original module

// Truth table for 2-to-4 decoder (S1, S0 -> Y0, Y1, Y2, Y3)
// S1 S0 | Y0 Y1 Y2 Y3
//  0  0 |  1  0  0  0  (Motor 0)
//  0  1 |  0  1  0  0  (Motor 1)
//  1  0 |  0  0  1  0  (Motor 2)
//  1  1 |  0  0  0  1  (Motor 3)

// =============================
#ifndef DIR_MULTIPLEXER_H
#define DIR_MULTIPLEXER_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <stdint.h>

// ---- Compile‑time switches
#ifndef DIRMUX_FAST_IO
    #define DIRMUX_FAST_IO 1  // 1: use direct registers (fastest). 0: use gpio_set_level
#endif

#ifndef DIRMUX_LOG
    #define DIRMUX_LOG 0  // 1: enable log prints (log_d/log_e). Keep 0 in RT builds.
#endif

#if DIRMUX_LOG
    #include <esp_log.h>
    #define DMX_LOGD(fmt, ...) ESP_LOGD("DIRMUX", fmt, ##__VA_ARGS__)
    #define DMX_LOGE(fmt, ...) ESP_LOGE("DIRMUX", fmt, ##__VA_ARGS__)
#else
    #define DMX_LOGD(...) ((void)0)
    #define DMX_LOGE(...) ((void)0)
#endif

class DirMultiplexer
{
public:
    DirMultiplexer(uint8_t s0Pin, uint8_t s1Pin, uint8_t dirPin) noexcept;

    bool begin() noexcept;                       // idempotent init; sets safe state (00, DIR=0)
    bool selectMotor(uint8_t motorId) noexcept;  // 0..3; returns false if invalid/not inited
    void setDirection(bool direction) noexcept;  // true=HIGH (FWD), false=LOW (REV)
    bool setMotorDirection(uint8_t motorId,
                           bool    direction) noexcept;  // select + set dir

    // Hot‑path unchecked variants (for already‑validated IDs). ISR‑safe.
    inline void IRAM_ATTR selectMotorUnchecked(uint8_t motorId) noexcept
    {
        setSelectPins(motorId);
        _currentMotor = motorId;
    }
    inline void IRAM_ATTR setDirectionUnchecked(bool direction) noexcept
    {
#if DIRMUX_FAST_IO
        fastWrite(_dirPin, direction);
#else
        gpio_set_level(static_cast<gpio_num_t>(_dirPin), direction ? 1 : 0);
#endif
        _currentDirection = direction;
    }

    // State
    inline uint8_t getCurrentMotor() const noexcept
    {
        return _currentMotor;
    }
    inline bool getCurrentDirection() const noexcept
    {
        return _currentDirection;
    }

    // Place S0/S1=HIGH, DIR=LOW (depends on external decoder; mirrors your
    // original behavior)
    void disable() noexcept;

    // Lightweight self‑test (optional; no delays). Returns true on pass.
    bool test(bool printResults) noexcept;

private:
    inline bool isValidMotorId(uint8_t motorId) const noexcept
    {
        return motorId < 4;
    }
    inline void IRAM_ATTR setSelectPins(uint8_t motorId) noexcept;

#if DIRMUX_FAST_IO
    // Fast write helpers (handle ports 0 and 1)
    static inline void IRAM_ATTR fastSet(uint8_t pinMaskPort0, uint8_t pinMaskPort1) noexcept
    {
        if (pinMaskPort0)
            GPIO.out_w1ts = pinMaskPort0;
    #if SOC_GPIO_VALID_GPIO_MASK > 0xFFFFFFFF
        if (pinMaskPort1)
            GPIO.out1_w1ts.val = pinMaskPort1;
    #else
        if (pinMaskPort1)
            GPIO.out1_w1ts.val = pinMaskPort1;  // kept for classic ESP32
    #endif
    }
    static inline void IRAM_ATTR fastClr(uint32_t maskP0, uint32_t maskP1) noexcept
    {
        if (maskP0)
            GPIO.out_w1tc = maskP0;
        if (maskP1)
            GPIO.out1_w1tc.val = maskP1;
    }
    static inline void IRAM_ATTR fastWrite(uint8_t pin, bool level) noexcept
    {
        if (pin < 32)
        {
            const uint32_t m = 1UL << pin;
            if (level)
                GPIO.out_w1ts = m;
            else
                GPIO.out_w1tc = m;
        }
        else
        {
            const uint32_t m = 1UL << (pin - 32);
            if (level)
                GPIO.out1_w1ts.val = m;
            else
                GPIO.out1_w1tc.val = m;
        }
    }
#endif

private:
    uint8_t _s0Pin;
    uint8_t _s1Pin;
    uint8_t _dirPin;
    uint8_t _currentMotor;
    bool    _currentDirection;
    bool    _initialized;
};

#endif  // DIR_MULTIPLEXER_H
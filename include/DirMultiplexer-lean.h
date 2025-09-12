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
        #define DIRMUX_FAST_IO                                                 \
            1  // 1: use direct registers (fastest). 0: use gpio_set_level
    #endif

    #ifndef DIRMUX_LOG
        #define DIRMUX_LOG                                                     \
            0  // 1: enable log prints (log_d/log_e). Keep 0 in RT builds.
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

    bool begin() noexcept;  // idempotent init; sets safe state (00, DIR=0)
    bool selectMotor(
        uint8_t motorId) noexcept;  // 0..3; returns false if invalid/not inited
    void
    setDirection(bool direction) noexcept;  // true=HIGH (FWD), false=LOW (REV)
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
    static inline void IRAM_ATTR fastSet(uint8_t pinMaskPort0,
                                         uint8_t pinMaskPort1) noexcept
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
    static inline void IRAM_ATTR fastClr(uint32_t maskP0,
                                         uint32_t maskP1) noexcept
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

// =============================
// File: DirMultiplexer.cpp
// =============================
#include "DirMultiplexer.h"

DirMultiplexer::DirMultiplexer(uint8_t s0Pin,
                               uint8_t s1Pin,
                               uint8_t dirPin) noexcept
    : _s0Pin(s0Pin),
      _s1Pin(s1Pin),
      _dirPin(dirPin),
      _currentMotor(0),
      _currentDirection(false),
      _initialized(false)
{
}

bool DirMultiplexer::begin() noexcept
{
    if (_initialized)
        return true;

    // Basic pin sanity (avoid input‑only pins ≥34)
    if (_s0Pin >= 34 || _s1Pin >= 34 || _dirPin >= 34)
    {
        DMX_LOGE("Invalid pins: s0=%u s1=%u dir=%u (>=34 are input‑only)",
                 _s0Pin,
                 _s1Pin,
                 _dirPin);
        return false;
    }

    gpio_config_t io{};
    io.mode         = GPIO_MODE_OUTPUT;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;

    io.pin_bit_mask = (1ULL << _s0Pin) | (1ULL << _s1Pin) | (1ULL << _dirPin);
    gpio_config(&io);

    // Safe default: select motor 0, DIR=LOW
#if DIRMUX_FAST_IO
    fastClr((1UL << _s0Pin) | (1UL << _s1Pin) | (1UL << _dirPin), 0);
#else
    gpio_set_level((gpio_num_t)_s0Pin, 0);
    gpio_set_level((gpio_num_t)_s1Pin, 0);
    gpio_set_level((gpio_num_t)_dirPin, 0);
#endif

    _currentMotor     = 0;
    _currentDirection = false;
    _initialized      = true;
    DMX_LOGD("DIRMUX init OK (S0=%u S1=%u DIR=%u)", _s0Pin, _s1Pin, _dirPin);
    return true;
}

bool DirMultiplexer::selectMotor(uint8_t motorId) noexcept
{
    if (!_initialized)
    {
        DMX_LOGE("Not initialized");
        return false;
    }
    if (!isValidMotorId(motorId))
    {
        DMX_LOGE("Invalid motorId=%u", motorId);
        return false;
    }
    setSelectPins(motorId);
    _currentMotor = motorId;
    return true;
}

void DirMultiplexer::setDirection(bool direction) noexcept
{
    if (!_initialized)
    {
        DMX_LOGE("Not initialized");
        return;
    }
#if DIRMUX_FAST_IO
    fastWrite(_dirPin, direction);
#else
    gpio_set_level(static_cast<gpio_num_t>(_dirPin), direction ? 1 : 0);
#endif
    _currentDirection = direction;
}

bool DirMultiplexer::setMotorDirection(uint8_t motorId, bool direction) noexcept
{
    if (!selectMotor(motorId))
        return false;
    setDirection(direction);
    return true;
}

void DirMultiplexer::disable() noexcept
{
    if (!_initialized)
        return;
    // Mirror legacy behavior: S0=S1=HIGH, DIR=LOW
#if DIRMUX_FAST_IO
    // clear DIR, set S0/S1
    if (_dirPin < 32)
        GPIO.out_w1tc = (1UL << _dirPin);
    else
        GPIO.out1_w1tc.val = (1UL << (_dirPin - 32));
    uint32_t set0 = 0, set1 = 0;
    if (_s0Pin < 32)
        set0 |= (1UL << _s0Pin);
    else
        set1 |= (1UL << (_s0Pin - 32));
    if (_s1Pin < 32)
        set0 |= (1UL << _s1Pin);
    else
        set1 |= (1UL << (_s1Pin - 32));
    if (set0)
        GPIO.out_w1ts = set0;
    if (set1)
        GPIO.out1_w1ts.val = set1;
#else
    gpio_set_level((gpio_num_t)_dirPin, 0);
    gpio_set_level((gpio_num_t)_s0Pin, 1);
    gpio_set_level((gpio_num_t)_s1Pin, 1);
#endif
}

bool DirMultiplexer::test(bool printResults) noexcept
{
    if (!_initialized)
    {
        if (printResults)
            DMX_LOGE("Test: not initialized");
        return false;
    }
    bool ok = true;
    for (uint8_t m = 0; m < 4; ++m)
    {
        if (!selectMotor(m))
        {
            ok = false;
            if (printResults)
                DMX_LOGE("Select %u failed", m);
            continue;
        }
        setDirection(true);
        setDirection(false);
    }
    // invalid id must fail
    if (selectMotor(4))
    {
        ok = false;
        if (printResults)
            DMX_LOGE("Invalid id accepted");
    }
    if (printResults)
        DMX_LOGD("DIRMUX test %s", ok ? "PASS" : "FAIL");
    return ok;
}

inline void IRAM_ATTR DirMultiplexer::setSelectPins(uint8_t motorId) noexcept
{
    // motorId bits: [S1 S0]
#if DIRMUX_FAST_IO
    // Build clear/set masks per port to update both pins with at most 4 writes
    uint32_t   clr0 = 0, clr1 = 0, set0 = 0, set1 = 0;
    const bool s0 = (motorId & 0x01) != 0;
    const bool s1 = (motorId & 0x02) != 0;

    if (_s0Pin < 32)
    {
        (s0 ? set0 : clr0) |= (1UL << _s0Pin);
    }
    else
    {
        (s0 ? set1 : clr1) |= (1UL << (_s0Pin - 32));
    }

    if (_s1Pin < 32)
    {
        (s1 ? set0 : clr0) |= (1UL << _s1Pin);
    }
    else
    {
        (s1 ? set1 : clr1) |= (1UL << (_s1Pin - 32));
    }

    if (clr0)
        GPIO.out_w1tc = clr0;
    if (set0)
        GPIO.out_w1ts = set0;
    if (clr1)
        GPIO.out1_w1tc.val = clr1;
    if (set1)
        GPIO.out1_w1ts.val = set1;
#else
    gpio_set_level((gpio_num_t)_s0Pin, (motorId & 0x01) ? 1 : 0);
    gpio_set_level((gpio_num_t)_s1Pin, (motorId & 0x02) ? 1 : 0);
#endif
}

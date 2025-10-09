#include "DirMultiplexer_lean.h"  // fast DIR mux

// =============================
// File: DirMultiplexer.cpp
// =============================
DirMultiplexer::DirMultiplexer(uint8_t s0Pin, uint8_t s1Pin, uint8_t dirPin) noexcept
    : _s0Pin(s0Pin), _s1Pin(s1Pin), _dirPin(dirPin), _currentMotor(0), _currentDirection(false), _initialized(false)
{
}

bool DirMultiplexer::begin() noexcept
{
    if (_initialized)
        return true;

    // Basic pin sanity (avoid input‑only pins ≥34)
    if (_s0Pin >= 34 || _s1Pin >= 34 || _dirPin >= 34)
    {
        DMX_LOGE("Invalid pins: s0=%u s1=%u dir=%u (>=34 are input‑only)", _s0Pin, _s1Pin, _dirPin);
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

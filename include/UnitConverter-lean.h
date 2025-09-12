// =============================
// File: UnitConverter.h
// Lean, allocation‑free, RT‑friendly rewrite
// - Same API & types as your existing module
// - Fewer divisions, no while‑loops, noexcept everywhere
// - Angle math uses fmod‑based normalization (branch‑light)
// =============================
#ifndef UNIT_CONVERTER_H
    #define UNIT_CONVERTER_H

    #include <cmath>
    #include <cstdint>

    // ---- Compatibility fallbacks (will be ignored if already defined upstream)
    #ifndef ENCODER_RESOLUTION
        #define ENCODER_RESOLUTION 4096
    #endif
    #ifndef LEAD_SCREW_PITCH_UM
        #define LEAD_SCREW_PITCH_UM 800.0
    #endif
    #ifndef MICROSTEPS_64
        #define MICROSTEPS_64 64
    #endif

enum class MotorType : std::uint8_t
{
    ROTATIONAL = 0,
    LINEAR     = 1
};

struct ConvertValues
{
    // Same field types/names as your original module
    std::int32_t  TO_TURNS{0};
    std::double_t TO_DEGREES{0};
    std::double_t TO_MICROMETERS{0};
    std::double_t TO_PULSES{0};
    std::int32_t  TO_STEPS{0};
};

class UnitConverter
{
public:
    // ---- Conversions (pure, allocation‑free)
    static ConvertValues convertFromDegrees(std::double_t degrees) noexcept;
    static ConvertValues convertFromPulses(std::double_t pulses_f) noexcept;
    static ConvertValues convertFromSteps(std::int32_t steps) noexcept;
    static ConvertValues convertFromMicrometers(std::double_t micrometers) noexcept;

    // ---- Angle helpers
    static inline std::double_t wrapAngle(std::double_t angle) noexcept
    {
        // Normalize to [0,360)
        // fmod for signed values: ((a mod 360) + 360) mod 360
        const std::double_t m = std::fmod(angle, k360);
        return std::fmod(m + k360, k360);
    }

    static inline std::double_t calculateShortestPath(std::double_t currentAngle, std::double_t targetAngle) noexcept
    {
        // Result in [-180, +180]
        const std::double_t a = wrapAngle(targetAngle) - wrapAngle(currentAngle);
        const std::double_t d = std::fmod(a + k180, k360) - k180;
        // Handle the edge where fmod returns -180 exactly, keep behavior of original
        return (d < -k180) ? (d + k360) : (d > k180 ? (d - k360) : d);
    }

    // ---- Configuration (same names)
    static inline void setDefaultMotorType(MotorType motorType) noexcept
    {
        _defaultMotorType = motorType;
    }
    static inline void setDefaultMicrosteps(std::int32_t microsteps) noexcept
    {
        _defaultMicrosteps = microsteps;
    }
    static inline void setDefaultResolution(std::int32_t resolution) noexcept
    {
        _defaultResolution_f = static_cast<std::double_t>(resolution);
    }
    static inline void setDefaultMicrometers(std::double_t micrometers) noexcept
    {
        _defaultMicrometers = micrometers;
    }

    static inline std::int32_t getDefaultMicrosteps() noexcept
    {
        return _defaultMicrosteps;
    }
    static inline std::double_t getDefaultResolution() noexcept
    {
        return _defaultResolution_f;
    }
    static inline std::double_t getDefaultMicrometers() noexcept
    {
        return _defaultMicrometers;
    }

    static inline bool isLinear() noexcept
    {
        return _defaultMotorType == MotorType::LINEAR;
    }
    static inline bool isRotational() noexcept
    {
        return _defaultMotorType == MotorType::ROTATIONAL;
    }

private:
    // Constants kept local to avoid global divisions
    static constexpr std::double_t k360 = 360.0;
    static constexpr std::double_t k180 = 180.0;

    // Defaults (match your original behavior)
    static MotorType     _defaultMotorType;
    static std::int32_t  _defaultMicrosteps;
    static std::double_t _defaultResolution_f;
    static std::double_t _defaultMicrometers;
};

#endif  // UNIT_CONVERTER_H

// =============================
// File: UnitConverter.cpp
// =============================
#include "UnitConverter.h"

// ---- Static member initialization (kept compatible with your originals)
MotorType     UnitConverter::_defaultMotorType    = MotorType::ROTATIONAL;
std::int32_t  UnitConverter::_defaultMicrosteps   = (MICROSTEPS_64 - 1) * 200;  // preserve legacy calc
std::double_t UnitConverter::_defaultResolution_f = static_cast<std::double_t>(ENCODER_RESOLUTION);
std::double_t UnitConverter::_defaultMicrometers  = LEAD_SCREW_PITCH_UM;

static inline std::int32_t fast_floor_to_i32(std::double_t x) noexcept
{
    // Faster than static_cast<int>(std::floor()) for non‑NaN typical ranges
    const auto i = static_cast<std::int32_t>(x);
    return (x < static_cast<std::double_t>(i)) ? (i - 1) : i;
}

ConvertValues UnitConverter::convertFromDegrees(std::double_t degrees) noexcept
{
    ConvertValues out{};

    // Turns + residual degrees (branch‑light)
    const std::double_t t = degrees / k360;
    out.TO_TURNS          = fast_floor_to_i32(t);
    out.TO_DEGREES        = degrees - static_cast<std::double_t>(out.TO_TURNS) * k360;
    if (isRotational())
        out.TO_TURNS = 0;

    // Scale factors (avoid multiple divisions by pulling out constants)
    const std::double_t deg_to_um   = _defaultMicrometers / k360;
    const std::double_t deg_to_puls = _defaultResolution_f / k360;
    const std::double_t deg_to_step = static_cast<std::double_t>(_defaultMicrosteps) / k360;

    out.TO_MICROMETERS = out.TO_DEGREES * deg_to_um;
    out.TO_PULSES      = out.TO_DEGREES * deg_to_puls;
    out.TO_STEPS       = static_cast<std::int32_t>(std::lround(out.TO_DEGREES * deg_to_step));
    return out;
}

ConvertValues UnitConverter::convertFromPulses(std::double_t pulses_f) noexcept
{
    ConvertValues out{};

    const std::double_t turns_d = pulses_f / _defaultResolution_f;
    out.TO_TURNS                = fast_floor_to_i32(turns_d);
    out.TO_PULSES               = pulses_f - static_cast<std::double_t>(out.TO_TURNS) * _defaultResolution_f;
    if (isRotational())
        out.TO_TURNS = 0;

    const std::double_t p_to_um   = _defaultMicrometers / _defaultResolution_f;
    const std::double_t p_to_deg  = k360 / _defaultResolution_f;
    const std::double_t p_to_step = static_cast<std::double_t>(_defaultMicrosteps) / _defaultResolution_f;

    out.TO_MICROMETERS = out.TO_PULSES * p_to_um;
    out.TO_DEGREES     = out.TO_PULSES * p_to_deg;
    out.TO_STEPS       = static_cast<std::int32_t>(std::lround(out.TO_PULSES * p_to_step));
    return out;
}

ConvertValues UnitConverter::convertFromSteps(std::int32_t steps) noexcept
{
    ConvertValues out{};

    const std::int32_t ms = _defaultMicrosteps;
    out.TO_TURNS          = steps / ms;  // integer division
    out.TO_STEPS          = steps % ms;  // remainder within one rev
    if (isRotational())
        out.TO_TURNS = 0;

    const std::double_t steps_d     = static_cast<std::double_t>(out.TO_STEPS);
    const std::double_t step_to_um  = _defaultMicrometers / static_cast<std::double_t>(ms);
    const std::double_t step_to_pul = _defaultResolution_f / static_cast<std::double_t>(ms);
    const std::double_t step_to_deg = k360 / static_cast<std::double_t>(ms);

    out.TO_MICROMETERS = steps_d * step_to_um;
    out.TO_PULSES      = steps_d * step_to_pul;
    out.TO_DEGREES     = steps_d * step_to_deg;
    return out;
}

ConvertValues UnitConverter::convertFromMicrometers(std::double_t micrometers) noexcept
{
    ConvertValues out{};

    const std::double_t turns_d = micrometers / _defaultMicrometers;
    out.TO_TURNS                = fast_floor_to_i32(turns_d);
    out.TO_MICROMETERS          = micrometers - static_cast<std::double_t>(out.TO_TURNS) * _defaultMicrometers;
    if (isRotational())
        out.TO_TURNS = 0;

    const std::double_t um_to_pul = _defaultResolution_f / _defaultMicrometers;
    const std::double_t um_to_deg = k360 / _defaultMicrometers;
    const std::double_t um_to_stp = static_cast<std::double_t>(_defaultMicrosteps) / _defaultMicrometers;

    out.TO_PULSES  = out.TO_MICROMETERS * um_to_pul;
    out.TO_STEPS   = static_cast<std::int32_t>(std::lround(out.TO_MICROMETERS * um_to_stp));
    out.TO_DEGREES = out.TO_MICROMETERS * um_to_deg;
    return out;
}

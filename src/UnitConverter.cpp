#include "UnitConverter_lean.h"  // unit conversions

// =============================
// File: UnitConverter.cpp
// =============================
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
    // out.TO_DEGREES        = degrees - static_cast<std::double_t>(out.TO_TURNS) * k360;
    if (isRotational())
        out.TO_TURNS = 0;

    // Scale factors (avoid multiple divisions by pulling out constants)
    const std::double_t deg_to_um   = _defaultMicrometers / k360;
    const std::double_t deg_to_puls = _defaultResolution_f / k360;
    const std::double_t deg_to_step = static_cast<std::double_t>(_defaultMicrosteps) / k360;

    out.TO_MICROMETERS = degrees * deg_to_um;
    out.TO_PULSES      = degrees * deg_to_puls;
    out.TO_STEPS       = static_cast<std::int32_t>(std::lround(degrees * deg_to_step));
    return out;
}

ConvertValues UnitConverter::convertFromPulses(std::double_t pulses_f) noexcept
{
    ConvertValues out{};

    const std::double_t turns_d = pulses_f / _defaultResolution_f;
    out.TO_TURNS                = fast_floor_to_i32(turns_d);
    // out.TO_PULSES               = pulses_f - static_cast<std::double_t>(out.TO_TURNS) * _defaultResolution_f;
    if (isRotational())
        out.TO_TURNS = 0;

    const std::double_t p_to_um   = _defaultMicrometers / _defaultResolution_f;
    const std::double_t p_to_deg  = k360 / _defaultResolution_f;
    const std::double_t p_to_step = static_cast<std::double_t>(_defaultMicrosteps) / _defaultResolution_f;

    out.TO_MICROMETERS = pulses_f * p_to_um;
    out.TO_DEGREES     = pulses_f * p_to_deg;
    out.TO_STEPS       = static_cast<std::int32_t>(std::lround(pulses_f * p_to_step));
    return out;
}

ConvertValues UnitConverter::convertFromSteps(std::int32_t steps) noexcept
{
    ConvertValues out{};

    const std::int32_t ms = _defaultMicrosteps;
    out.TO_TURNS          = steps / ms;  // integer division
    // out.TO_STEPS          = steps % ms;  // remainder within one rev
    if (isRotational())
        out.TO_TURNS = 0;

    const std::double_t steps_d     = static_cast<std::double_t>(steps);
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
    // out.TO_MICROMETERS          = micrometers - static_cast<std::double_t>(out.TO_TURNS) * _defaultMicrometers;
    if (isRotational())
        out.TO_TURNS = 0;

    const std::double_t um_to_pul = _defaultResolution_f / _defaultMicrometers;
    const std::double_t um_to_deg = k360 / _defaultMicrometers;
    const std::double_t um_to_stp = static_cast<std::double_t>(_defaultMicrosteps) / _defaultMicrometers;

    out.TO_PULSES  = micrometers * um_to_pul;
    out.TO_STEPS   = static_cast<std::int32_t>(std::lround(micrometers * um_to_stp));
    out.TO_DEGREES = micrometers * um_to_deg;
    return out;
}

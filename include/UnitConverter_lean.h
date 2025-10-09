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
#ifndef UNIT_CONVERTER_H
#define UNIT_CONVERTER_H

#include <Arduino.h>
#include <cmath>

// Physical constants for conversion calculations
static constexpr std::double_t PIXEL_SIZE_UM              = 5.2f;  // Size of each pixel in the camera (micrometers)
static constexpr std::double_t PIXEL_SIZE_MM              = (PIXEL_SIZE_UM * 1e-3f);
static constexpr std::double_t CAMERA_TO_MIRROR_LENGTH_MM = 195.0f;  // Distance from mirror to camera in millimeters
static constexpr std::double_t LEAD_SCREW_PITCH_UM        = 200.0f;  // Lead screw pitch in micrometers
static constexpr std::int32_t  ENCODER_RESOLUTION         = 4096;    // Encoder 12 bits
static constexpr uint16_t      MICROSTEPS_64              = 64;      // Default current in mA
static constexpr uint16_t      MICROSTEPS_32              = 32;      // Default current in mA

// Motor types for conversion logic
enum class MotorType : uint8_t
{
    LINEAR     = 0,
    ROTATIONAL = 1,
};

// Conversion result structures
struct ConvertValues
{
    std::double_t TO_MICROMETERS;
    std::double_t TO_PULSES;
    std::int32_t  TO_STEPS;
    std::double_t TO_DEGREES;
    std::int32_t  TO_TURNS;
};

/**
 * @brief Unit Converter Module - Singleton for motion control unit conversions
 *
 * This module provides static methods for converting between different units
 * used in motion control systems: degrees, pulses, steps, and micrometers.
 *
 * Features:
 * - Static methods for easy access without instantiation
 * - Support for both linear and rotational motor types
 * - Configurable parameters for different motor configurations
 * - High precision conversion calculations
 */
class UnitConverter
{
public:
    // Conversion methods from different units
    static ConvertValues convertFromDegrees(std::double_t degrees);
    static ConvertValues convertFromPulses(std::double_t pulses);
    static ConvertValues convertFromSteps(std::int32_t steps);
    static ConvertValues convertFromMicrometers(std::double_t umeters);

    // Utility methods
    static std::double_t wrapAngle(std::double_t angle);
    static std::double_t calculateShortestPath(std::double_t currentAngle, std::double_t targetAngle);

    // Configuration methods
    static void          setDefaultMotorType(MotorType motorType = MotorType::ROTATIONAL);
    static void          setDefaultMicrosteps(std::int32_t microsteps);
    static void          setDefaultResolution(std::int32_t resolution);
    static void          setDefaultMicrometers(std::double_t micrometers);
    static std::int32_t  getDefaultMicrosteps();
    static std::double_t getDefaultResolution();
    static std::double_t getDefaultMicrometers();
    static bool          isLinear();
    static bool          isRotational();

private:
    // Default configuration values
    static MotorType     _defaultMotorType;
    static std::int32_t  _defaultMicrosteps;
    static std::double_t _defaultResolution_f;
    static std::double_t _defaultMicrometers;

    // Private constructor to prevent instantiation (Singleton pattern)
    UnitConverter()                                = delete;
    ~UnitConverter()                               = delete;
    UnitConverter(const UnitConverter&)            = delete;
    UnitConverter& operator=(const UnitConverter&) = delete;
};

#endif  // UNIT_CONVERTER_H

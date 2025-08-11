#ifndef UNIT_CONVERTER_H
#define UNIT_CONVERTER_H

#include <Arduino.h>
#include <cmath>

// Physical constants for conversion calculations
static constexpr float    PIXEL_SIZE_UM              = 5.2f;  // Size of each pixel in the camera (micrometers)
static constexpr float    PIXEL_SIZE_MM              = (PIXEL_SIZE_UM * 1e-3f);
static constexpr float    CAMERA_TO_MIRROR_LENGTH_MM = 195.0f;  // Distance from mirror to camera in millimeters
static constexpr float    LEAD_SCREW_PITCH_UM        = 200.0f;  // Lead screw pitch in micrometers
static constexpr int32_t  ENCODER_RESOLUTION         = 4096;    // Encoder 12 bits
static constexpr uint16_t MICROSTEPS_64              = 64;      // Default current in mA
static constexpr uint16_t MICROSTEPS_32              = 32;      // Default current in mA

// Motor types for conversion logic
enum class MotorType
{
    LINEAR     = 0,
    ROTATIONAL = 1,
};

// Conversion result structures
struct ConvertValues
{
    struct FromDegrees
    {
        int32_t TO_PULSES;
        int32_t TO_STEPS;
        float   TO_UMETERS;
        int32_t TO_TURNS;
    };

    struct FromPulses
    {
        float   TO_DEGREES;
        int32_t TO_STEPS;
        float   TO_UMETERS;
        int32_t TO_TURNS;
    };

    struct FromSteps
    {
        int32_t TO_PULSES;
        float   TO_DEGREES;
        float   TO_UMETERS;
        int32_t TO_TURNS;
    };

    struct FromUMeters
    {
        int32_t TO_PULSES;
        int32_t TO_STEPS;
        float   TO_DEGREES;
        int32_t TO_TURNS;
    };
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
    static ConvertValues::FromDegrees convertFromDegrees(float degrees, int32_t microsteps = (MICROSTEPS_64 - 1) * 200, int32_t resolution = ENCODER_RESOLUTION, float micrometers = LEAD_SCREW_PITCH_UM, MotorType motorType = MotorType::ROTATIONAL);

    static ConvertValues::FromPulses convertFromPulses(int32_t pulses, int32_t microsteps = (MICROSTEPS_64 - 1) * 200, int32_t resolution = ENCODER_RESOLUTION, float micrometers = LEAD_SCREW_PITCH_UM, MotorType motorType = MotorType::ROTATIONAL);

    static ConvertValues::FromSteps convertFromSteps(int32_t steps, int32_t microsteps = (MICROSTEPS_64 - 1) * 200, int32_t resolution = ENCODER_RESOLUTION, float micrometers = LEAD_SCREW_PITCH_UM, MotorType motorType = MotorType::ROTATIONAL);

    static ConvertValues::FromUMeters convertFromUMeters(float umeters, int32_t microsteps = (MICROSTEPS_64 - 1) * 200, int32_t resolution = ENCODER_RESOLUTION, float micrometers = LEAD_SCREW_PITCH_UM, MotorType motorType = MotorType::ROTATIONAL);

    // Utility methods
    static float calculateMotorAngleFromReference(float newPixel, float refPixel, float refMotorDeg);
    static float wrapAngle(float angle);
    static float calculateShortestPath(float currentAngle, float targetAngle);

    // Configuration methods
    static void    setDefaultMicrosteps(int32_t microsteps);
    static void    setDefaultResolution(int32_t resolution);
    static void    setDefaultMicrometers(float micrometers);
    static int32_t getDefaultMicrosteps();
    static int32_t getDefaultResolution();
    static float   getDefaultMicrometers();

private:
    // Default configuration values
    static int32_t _defaultMicrosteps;
    static int32_t _defaultMicrosteps_32;
    static int32_t _defaultResolution;
    static float   _defaultMicrometers;

    // Private constructor to prevent instantiation (Singleton pattern)
    UnitConverter()                                = delete;
    ~UnitConverter()                               = delete;
    UnitConverter(const UnitConverter&)            = delete;
    UnitConverter& operator=(const UnitConverter&) = delete;
};

#endif  // UNIT_CONVERTER_H

#include "UnitConverter.h"
#include <cmath>

// Static member initialization
int32_t UnitConverter::_defaultMicrosteps  = (MICROSTEPS_64 - 1) * 200;
int32_t UnitConverter::_defaultResolution  = ENCODER_RESOLUTION;
float   UnitConverter::_defaultMicrometers = LEAD_SCREW_PITCH_UM;

ConvertValues::FromDegrees UnitConverter::convertFromDegrees(float degrees, int32_t microsteps, int32_t resolution, float micrometers, MotorType motorType)
{
    float                      remain = 0;
    ConvertValues::FromDegrees convert;

    if (motorType == MotorType::LINEAR)
    {
        setDefaultMicrosteps((MICROSTEPS_32 - 1) * 200);
        convert.TO_TURNS = static_cast<int32_t>(std::floor(degrees / 360.0f));
        remain           = degrees - (convert.TO_TURNS * 360.0f);
    }
    else
    {
        convert.TO_TURNS = 0;
        remain           = degrees;
    }

    convert.TO_PULSES  = static_cast<int32_t>(std::round(remain * (resolution / 360.0f)));
    convert.TO_STEPS   = static_cast<int32_t>(std::round(remain * (microsteps / 360.0f)));
    convert.TO_UMETERS = static_cast<float>(remain * (micrometers / 360.0f));

    return convert;
}

ConvertValues::FromPulses UnitConverter::convertFromPulses(int32_t pulses, int32_t microsteps, int32_t resolution, float micrometers, MotorType motorType)
{
    int32_t                   remain = 0;
    ConvertValues::FromPulses convert;

    if (motorType == MotorType::LINEAR)
    {
        setDefaultMicrosteps((MICROSTEPS_32 - 1) * 200);
        convert.TO_TURNS = pulses / resolution;
        remain           = pulses % resolution;
    }
    else
    {
        convert.TO_TURNS = 0;
        remain           = pulses;
    }

    convert.TO_DEGREES = static_cast<float>(remain * (360.0f / resolution));
    convert.TO_STEPS   = static_cast<int32_t>(std::round(remain * (microsteps / resolution)));
    convert.TO_UMETERS = static_cast<float>(remain * (micrometers / resolution));

    return convert;
}

ConvertValues::FromSteps UnitConverter::convertFromSteps(int32_t steps, int32_t microsteps, int32_t resolution, float micrometers, MotorType motorType)
{
    int32_t                  remain = 0;
    ConvertValues::FromSteps convert;

    if (motorType == MotorType::LINEAR)
    {
        setDefaultMicrosteps((MICROSTEPS_32 - 1) * 200);
        convert.TO_TURNS = steps / microsteps;
        remain           = steps % microsteps;
    }
    else
    {
        convert.TO_TURNS = 0;
        remain           = steps;
    }

    convert.TO_PULSES  = static_cast<int32_t>(std::round(remain * (resolution / microsteps)));
    convert.TO_DEGREES = static_cast<float>(remain * (360.0f / microsteps));
    convert.TO_UMETERS = static_cast<float>(remain * (micrometers / microsteps));

    return convert;
}

ConvertValues::FromUMeters UnitConverter::convertFromUMeters(float umeters, int32_t microsteps, int32_t resolution, float micrometers, MotorType motorType)
{
    float                      remain = 0;
    ConvertValues::FromUMeters convert;

    if (motorType == MotorType::LINEAR)
    {
        setDefaultMicrosteps((MICROSTEPS_32 - 1) * 200);
        convert.TO_TURNS = static_cast<int32_t>(std::floor(umeters / micrometers));
        remain           = umeters - (convert.TO_TURNS * micrometers);
    }
    else
    {
        convert.TO_TURNS = 0;
        remain           = umeters;
    }

    convert.TO_DEGREES = static_cast<float>(remain * (360.0f / micrometers));
    convert.TO_STEPS   = static_cast<int32_t>(std::round(remain * (microsteps / micrometers)));
    convert.TO_PULSES  = static_cast<int32_t>(std::round(remain * (resolution / micrometers)));

    return convert;
}

float UnitConverter::calculateMotorAngleFromReference(float newPixel, float refPixel, float refMotorDeg)
{
    float deltaPixel     = newPixel - refPixel;
    float delta_mm       = deltaPixel * PIXEL_SIZE_MM;
    float mirrorAngleRad = atan(delta_mm / CAMERA_TO_MIRROR_LENGTH_MM);
    float motorAngleDeg  = refMotorDeg + (mirrorAngleRad * 180.0f / M_PI);

    return motorAngleDeg;
}

float UnitConverter::wrapAngle(float angle)
{
    // Normalize angle to 0-360 range
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }
    return angle;
}

float UnitConverter::calculateShortestPath(float currentAngle, float targetAngle)
{
    // Normalize both angles to 0-360 range
    currentAngle = wrapAngle(currentAngle);
    targetAngle  = wrapAngle(targetAngle);

    // Calculate the difference
    float diff = targetAngle - currentAngle;

    // Find the shortest path
    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    else if (diff < -180.0f)
    {
        diff += 360.0f;
    }

    return diff;
}

// Configuration methods
void UnitConverter::setDefaultMicrosteps(int32_t microsteps)
{
    _defaultMicrosteps = microsteps;
}

void UnitConverter::setDefaultResolution(int32_t resolution)
{
    _defaultResolution = resolution;
}

void UnitConverter::setDefaultMicrometers(float micrometers)
{
    _defaultMicrometers = micrometers;
}

int32_t UnitConverter::getDefaultMicrosteps()
{
    return _defaultMicrosteps;
}

int32_t UnitConverter::getDefaultResolution()
{
    return _defaultResolution;
}

float UnitConverter::getDefaultMicrometers()
{
    return _defaultMicrometers;
}

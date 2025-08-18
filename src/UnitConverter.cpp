#include "UnitConverter.h"
#include <cmath>

// Static member initialization
MotorType UnitConverter::_defaultMotorType   = MotorType::ROTATIONAL;
int32_t   UnitConverter::_defaultMicrosteps  = (MICROSTEPS_64 - 1) * 200;
int32_t   UnitConverter::_defaultResolution  = ENCODER_RESOLUTION;
float     UnitConverter::_defaultMicrometers = LEAD_SCREW_PITCH_UM;

ConvertValues UnitConverter::convertFromDegrees(float degrees)
{
    ConvertValues convert;
    convert.TO_TURNS   = static_cast<int32_t>(std::floor(degrees / 360.0f));
    convert.TO_DEGREES = degrees - (convert.TO_TURNS * 360.0f);

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_MICROMETERS = static_cast<float>(convert.TO_DEGREES * (_defaultMicrometers / 360.0f));
    convert.TO_PULSES      = static_cast<int32_t>(std::round(convert.TO_DEGREES * (static_cast<float>(_defaultResolution) / 360.0f)));
    convert.TO_STEPS       = static_cast<int32_t>(std::round(convert.TO_DEGREES * (static_cast<float>(_defaultMicrosteps) / 360.0f)));

    return convert;
}

ConvertValues UnitConverter::convertFromPulses(int32_t pulses)
{
    ConvertValues convert;
    convert.TO_TURNS  = pulses / _defaultResolution;
    convert.TO_PULSES = pulses % _defaultResolution;

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_MICROMETERS = static_cast<float>(convert.TO_PULSES * (_defaultMicrometers / static_cast<float>(_defaultResolution)));
    convert.TO_DEGREES     = static_cast<float>(convert.TO_PULSES * (360.0f / static_cast<float>(_defaultResolution)));
    convert.TO_STEPS       = static_cast<int32_t>(std::round(convert.TO_PULSES * (static_cast<float>(_defaultMicrosteps) / static_cast<float>(_defaultResolution))));

    return convert;
}

ConvertValues UnitConverter::convertFromSteps(int32_t steps)
{
    ConvertValues convert;
    convert.TO_TURNS = steps / _defaultMicrosteps;
    convert.TO_STEPS = steps % _defaultMicrosteps;

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_MICROMETERS = static_cast<float>(convert.TO_STEPS * (_defaultMicrometers / static_cast<float>(_defaultMicrosteps)));
    convert.TO_PULSES      = static_cast<int32_t>(std::round(convert.TO_STEPS * (static_cast<float>(_defaultResolution) / static_cast<float>(_defaultMicrosteps))));
    convert.TO_DEGREES     = static_cast<float>(convert.TO_STEPS * (360.0f / static_cast<float>(_defaultMicrosteps)));

    return convert;
}

ConvertValues UnitConverter::convertFromMicrometers(float micrometers)
{
    ConvertValues convert;
    convert.TO_TURNS       = static_cast<int32_t>(std::floor(micrometers / _defaultMicrometers));
    convert.TO_MICROMETERS = micrometers - (convert.TO_TURNS * _defaultMicrometers);

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_PULSES  = static_cast<int32_t>(std::round(convert.TO_MICROMETERS * (static_cast<float>(_defaultResolution) / _defaultMicrometers)));
    convert.TO_STEPS   = static_cast<int32_t>(std::round(convert.TO_MICROMETERS * (static_cast<float>(_defaultMicrosteps) / _defaultMicrometers)));
    convert.TO_DEGREES = static_cast<float>(convert.TO_MICROMETERS * (360.0f / _defaultMicrometers));

    return convert;
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
void UnitConverter::setDefaultMotorType(MotorType motorType)
{
    _defaultMotorType = motorType;
}
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
bool UnitConverter::isLinear()
{
    return (_defaultMotorType == MotorType::LINEAR);
}
bool UnitConverter::isRotational()
{
    return (_defaultMotorType == MotorType::ROTATIONAL);
}
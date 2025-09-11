#include "UnitConverter.h"
#include <cmath>

// Static member initialization
MotorType     UnitConverter::_defaultMotorType    = MotorType::ROTATIONAL;
std::int32_t  UnitConverter::_defaultMicrosteps   = (MICROSTEPS_64 - 1) * 200;
std::double_t UnitConverter::_defaultResolution_f = static_cast<std::double_t>(ENCODER_RESOLUTION);
std::double_t UnitConverter::_defaultMicrometers  = LEAD_SCREW_PITCH_UM;

ConvertValues UnitConverter::convertFromDegrees(std::double_t degrees)
{
    ConvertValues convert;
    convert.TO_TURNS   = static_cast<std::int32_t>(std::floor(degrees / 360.0f));
    convert.TO_DEGREES = degrees - (convert.TO_TURNS * 360.0f);

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_MICROMETERS = static_cast<std::double_t>(convert.TO_DEGREES * (_defaultMicrometers / 360.0f));
    convert.TO_PULSES      = static_cast<std::int32_t>(std::round(convert.TO_DEGREES * (_defaultResolution_f / 360.0f)));
    convert.TO_STEPS       = static_cast<std::int32_t>(std::round(convert.TO_DEGREES * (static_cast<std::double_t>(_defaultMicrosteps) / 360.0f)));

    return convert;
}

ConvertValues UnitConverter::convertFromPulses(std::double_t pulses_f)
{
    ConvertValues convert;
    convert.TO_TURNS  = pulses_f / _defaultResolution_f;
    convert.TO_PULSES = pulses_f - (convert.TO_TURNS * _defaultResolution_f);

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_MICROMETERS = convert.TO_PULSES * (_defaultMicrometers / _defaultResolution_f);
    convert.TO_DEGREES     = convert.TO_PULSES * (360.0f / _defaultResolution_f);
    convert.TO_STEPS       = static_cast<std::int32_t>(std::round(convert.TO_PULSES * (static_cast<std::double_t>(_defaultMicrosteps) / _defaultResolution_f)));

    return convert;
}

ConvertValues UnitConverter::convertFromSteps(std::int32_t steps)
{
    ConvertValues convert;
    convert.TO_TURNS = steps / _defaultMicrosteps;
    convert.TO_STEPS = steps % _defaultMicrosteps;

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_MICROMETERS = static_cast<std::double_t>(convert.TO_STEPS * (_defaultMicrometers / static_cast<std::double_t>(_defaultMicrosteps)));
    convert.TO_PULSES      = static_cast<std::int32_t>(std::round(convert.TO_STEPS * (_defaultResolution_f / static_cast<std::double_t>(_defaultMicrosteps))));
    convert.TO_DEGREES     = static_cast<std::double_t>(convert.TO_STEPS * (360.0f / static_cast<std::double_t>(_defaultMicrosteps)));

    return convert;
}

ConvertValues UnitConverter::convertFromMicrometers(std::double_t micrometers)
{
    ConvertValues convert;
    convert.TO_TURNS       = static_cast<std::int32_t>(std::floor(micrometers / _defaultMicrometers));
    convert.TO_MICROMETERS = micrometers - (convert.TO_TURNS * _defaultMicrometers);

    if (isRotational())
    {
        convert.TO_TURNS = 0;
    }

    convert.TO_PULSES  = static_cast<std::int32_t>(std::round(convert.TO_MICROMETERS * (_defaultResolution_f / _defaultMicrometers)));
    convert.TO_STEPS   = static_cast<std::int32_t>(std::round(convert.TO_MICROMETERS * (static_cast<std::double_t>(_defaultMicrosteps) / _defaultMicrometers)));
    convert.TO_DEGREES = static_cast<std::double_t>(convert.TO_MICROMETERS * (360.0f / _defaultMicrometers));

    return convert;
}

std::double_t UnitConverter::wrapAngle(std::double_t angle)
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

std::double_t UnitConverter::calculateShortestPath(std::double_t currentAngle, std::double_t targetAngle)
{
    // Normalize both angles to 0-360 range
    currentAngle = wrapAngle(currentAngle);
    targetAngle  = wrapAngle(targetAngle);

    // Calculate the difference
    std::double_t diff = targetAngle - currentAngle;

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
void UnitConverter::setDefaultMicrosteps(std::int32_t microsteps)
{
    _defaultMicrosteps = microsteps;
}

void UnitConverter::setDefaultResolution(std::int32_t resolution)
{
    _defaultResolution_f = static_cast<std::double_t>(resolution);
}

void UnitConverter::setDefaultMicrometers(std::double_t micrometers)
{
    _defaultMicrometers = micrometers;
}

std::int32_t UnitConverter::getDefaultMicrosteps()
{
    return _defaultMicrosteps;
}

std::double_t UnitConverter::getDefaultResolution()
{
    return _defaultResolution_f;
}

std::double_t UnitConverter::getDefaultMicrometers()
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
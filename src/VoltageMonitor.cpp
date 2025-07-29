#include "VoltageMonitor.h"
#include <esp_log.h>

VoltageMonitor::VoltageMonitor(uint8_t pin, MonitorMode mode, uint16_t threshold, uint16_t debounceMs)
    : _pin(pin),
      _mode(mode),
      _threshold(threshold),
      _debounceMs(debounceMs),
      _enabled(false),
      _initialized(false),
      _currentLevel(0),
      _lastVoltageOK(true),
      _dropDetected(false),
      _lastChangeTime(0),
      _debouncing(false),
      _callback(nullptr)
{
}

bool VoltageMonitor::begin()
{
    if (_initialized)
        return true;

    // Configure pin based on monitoring mode
    if (_mode == MonitorMode::DIGITAL_)
    {
        pinMode(_pin, INPUT);
        // Enable internal pull-up for digital monitoring (optional)
        // digitalWrite(_pin, HIGH);
    }
    else  // ANALOG mode
    {
        // For ESP32, analog pins are automatically configured
        // No additional setup needed for analogRead
    }

    // Read initial voltage level
    _currentLevel   = readVoltageLevel();
    _lastVoltageOK  = isVoltageLevelOK(_currentLevel);
    _lastChangeTime = millis();

    _initialized = true;
    _enabled     = true;

    log_d("VoltageMonitor initialized - Pin: %d, Mode: %s, Threshold: %d", _pin,
          (_mode == MonitorMode::DIGITAL_) ? "DIGITAL" : "ANALOG", _threshold);

    return true;
}

void VoltageMonitor::onDrop(VoltageDropCallback callback)
{
    _callback = callback;
    log_d("Voltage drop callback registered for pin %d", _pin);
}

void VoltageMonitor::update()
{
    if (!_enabled || !_initialized)
        return;

    // Read current voltage level
    uint16_t newLevel = readVoltageLevel();

    // Check if voltage level has changed
    if (newLevel != _currentLevel)
    {
        processVoltageChange(newLevel);
    }

    // Handle debouncing
    if (_debouncing)
    {
        uint32_t currentTime = millis();
        if (currentTime - _lastChangeTime >= _debounceMs)
        {
            _debouncing = false;

            // Check if voltage is still below threshold after debounce
            bool currentVoltageOK = isVoltageLevelOK(_currentLevel);

            // If voltage dropped from OK to not OK, trigger callback
            if (_lastVoltageOK && !currentVoltageOK)
            {
                _dropDetected  = true;
                _lastVoltageOK = false;

                log_w("Voltage drop detected on pin %d (level: %d)", _pin, _currentLevel);

                // Call user callback if registered
                if (_callback)
                {
                    _callback();
                }
            }
            else if (!_lastVoltageOK && currentVoltageOK)
            {
                // Voltage recovered
                _lastVoltageOK = true;
                log_d("Voltage recovered on pin %d (level: %d)", _pin, _currentLevel);
            }
        }
    }
}

uint16_t VoltageMonitor::getCurrentLevel() const
{
    return _currentLevel;
}

bool VoltageMonitor::isVoltageOK() const
{
    return _lastVoltageOK;
}

bool VoltageMonitor::wasDropDetected()
{
    return _dropDetected;
}

void VoltageMonitor::resetDropDetection()
{
    _dropDetected = false;
}

void VoltageMonitor::setThreshold(uint16_t threshold)
{
    _threshold = threshold;
    log_d("Threshold updated to %d for pin %d", _threshold, _pin);
}

void VoltageMonitor::setDebounceTime(uint16_t debounceMs)
{
    _debounceMs = debounceMs;
    log_d("Debounce time updated to %d ms for pin %d", _debounceMs, _pin);
}

void VoltageMonitor::setEnabled(bool enabled)
{
    _enabled = enabled;
    log_d("VoltageMonitor %s for pin %d", enabled ? "enabled" : "disabled", _pin);
}

bool VoltageMonitor::isEnabled() const
{
    return _enabled;
}

uint16_t VoltageMonitor::readVoltageLevel()
{
    if (_mode == MonitorMode::DIGITAL_)
    {
        return digitalRead(_pin);
    }
    else  // ANALOG mode
    {
        return analogRead(_pin);
    }
}

bool VoltageMonitor::isVoltageLevelOK(uint16_t level) const
{
    if (_mode == MonitorMode::DIGITAL_)
    {
        // For digital mode, HIGH (1) is considered OK, LOW (0) is voltage drop
        return level == HIGH;
    }
    else  // ANALOG mode
    {
        // For analog mode, check if level is above threshold
        return level >= _threshold;
    }
}

void VoltageMonitor::processVoltageChange(uint16_t newLevel)
{
    _currentLevel   = newLevel;
    _lastChangeTime = millis();
    _debouncing     = true;

    log_d("Voltage change detected on pin %d: %d", _pin, newLevel);
}
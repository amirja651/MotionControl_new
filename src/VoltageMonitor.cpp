#include "VoltageMonitor_lean.h"  // voltage monitor
// =============================
// File: VoltageMonitor.cpp
// =============================
VoltageMonitor::VoltageMonitor(std::uint8_t  pin,
                               MonitorMode   mode,
                               std::uint16_t threshold,
                               std::uint16_t debounceMs) noexcept
    : _pin(pin),
      _mode(mode),
      _threshold(threshold),
      _debounceMs(debounceMs),
      _debounceUs(static_cast<std::uint32_t>(debounceMs) * 1000U),
      _hysteresis(0),
      _enabled(false),
      _initialized(false),
      _currentLevel(0),
      _lastVoltageOK(true),
      _dropDetected(false),
      _lastChangeTimeUs(0),
      _debouncing(false),
      _callback(nullptr)
{
}

bool VoltageMonitor::begin() noexcept
{
    if (_initialized)
        return true;

    if (_mode == MonitorMode::DIGITAL_)
    {
        // Configure as input; avoid pull‑ups unless required by hardware (leave it to board design)
        pinMode(_pin, INPUT);
    }
    else
    {  // ANALOG
       // Arduino‑ESP32 configures ADC pins on demand; nothing to do.
    }

    _currentLevel     = readVoltageLevel();
    _lastVoltageOK    = isVoltageLevelOK(_currentLevel);
    _lastChangeTimeUs = static_cast<std::uint64_t>(esp_timer_get_time());

    _initialized = true;
    _enabled     = true;
    VM_LOGD("VoltageMonitor init pin=%u mode=%s thr=%u deb=%u ms",
            _pin,
            (_mode == MonitorMode::DIGITAL_) ? "DIGITAL" : "ANALOG",
            _threshold,
            _debounceMs);
    return true;
}

void VoltageMonitor::onDrop(VoltageDropCallback cb) noexcept
{
    _callback = cb;
}

void VoltageMonitor::update() noexcept
{
    if (!_enabled || !_initialized)
        return;

    const std::uint16_t newLevel = readVoltageLevel();
    if (newLevel != _currentLevel)
    {
        processVoltageChange(newLevel);
    }

    if (_debouncing)
    {
        const std::uint64_t now = static_cast<std::uint64_t>(esp_timer_get_time());
        if ((now - _lastChangeTimeUs) >= _debounceUs)
        {
            _debouncing = false;

            const bool currentOK = isVoltageLevelOK(_currentLevel);
            if (_lastVoltageOK && !currentOK)
            {
                _dropDetected  = true;
                _lastVoltageOK = false;
                VM_LOGW("Voltage drop pin %u level=%u", _pin, _currentLevel);
                if (_callback)
                    _callback();
            }
            else if (!_lastVoltageOK && currentOK)
            {
                _lastVoltageOK = true;
                VM_LOGD("Voltage recovered pin %u level=%u", _pin, _currentLevel);
            }
        }
    }
}

bool VoltageMonitor::wasDropDetected() noexcept
{
    const bool v = _dropDetected;
    return v;
}

void VoltageMonitor::setThreshold(std::uint16_t threshold) noexcept
{
    _threshold = threshold;
    VM_LOGD("Threshold=%u pin=%u", _threshold, _pin);
}

void VoltageMonitor::setDebounceTime(std::uint16_t debounceMs) noexcept
{
    _debounceMs = debounceMs;
    _debounceUs = static_cast<std::uint32_t>(debounceMs) * 1000U;
    VM_LOGD("Debounce=%u ms pin=%u", _debounceMs, _pin);
}

inline std::uint16_t VoltageMonitor::readVoltageLevel() noexcept
{
    if (_mode == MonitorMode::DIGITAL_)
    {
#if VOLTMON_FAST_DIGITAL
        return static_cast<std::uint16_t>(gpio_get_level(static_cast<gpio_num_t>(_pin)));
#else
        return static_cast<std::uint16_t>(digitalRead(_pin));
#endif
    }
    else
    {
        return static_cast<std::uint16_t>(analogRead(_pin));
    }
}

inline bool VoltageMonitor::isVoltageLevelOK(std::uint16_t level) const noexcept
{
    if (_mode == MonitorMode::DIGITAL_)
    {
        // HIGH = OK, LOW = drop
        return level == HIGH;
    }
    // ANALOG with optional hysteresis window
    if (_hysteresis == 0)
    {
        return level >= _threshold;
    }
    else
    {
        const std::uint16_t lo = (_threshold > _hysteresis) ? static_cast<std::uint16_t>(_threshold - _hysteresis)
                                                            : static_cast<std::uint16_t>(0);
        const std::uint16_t hi = static_cast<std::uint16_t>(_threshold + _hysteresis);
        if (level <= lo)
            return false;  // definitely not OK
        if (level >= hi)
            return true;        // definitely OK
        return _lastVoltageOK;  // inside window → keep previous state
    }
}

inline void VoltageMonitor::processVoltageChange(std::uint16_t newLevel) noexcept
{
    _currentLevel     = newLevel;
    _lastChangeTimeUs = static_cast<std::uint64_t>(esp_timer_get_time());
    _debouncing       = true;
    VM_LOGD("Voltage change pin %u -> %u", _pin, newLevel);
}

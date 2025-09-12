// =============================
// File: VoltageMonitor.h
// Lean, allocation‑free, RT‑friendly rewrite for ESP32 (Arduino)
// - Same public API as your original module
// - Debounce in microseconds via esp_timer (no millis rollover)
// - Optional analog hysteresis (Schmitt), optional fast digital reads
// - No dynamic allocation; logging is compile‑time gated
// =============================
#ifndef VOLTAGE_MONITOR_H
    #define VOLTAGE_MONITOR_H

    #include <Arduino.h>
    #include <cstdint>
    #include <driver/gpio.h>
    #include <esp_timer.h>

    // ---- Compile‑time switches
    #ifndef VOLTMON_LOG
        #define VOLTMON_LOG 0  // 1: enable ESP_LOG* prints
    #endif
    #ifndef VOLTMON_FAST_DIGITAL
        #define VOLTMON_FAST_DIGITAL 1  // 1: use gpio_get_level / direct fast path
    #endif

    #if VOLTMON_LOG
        #include <esp_log.h>
        #define VM_LOGD(fmt, ...) ESP_LOGD("VMON", fmt, ##__VA_ARGS__)
        #define VM_LOGW(fmt, ...) ESP_LOGW("VMON", fmt, ##__VA_ARGS__)
        #define VM_LOGI(fmt, ...) ESP_LOGI("VMON", fmt, ##__VA_ARGS__)
    #else
        #define VM_LOGD(...) ((void)0)
        #define VM_LOGW(...) ((void)0)
        #define VM_LOGI(...) ((void)0)
    #endif

// Keep legacy names for compatibility
enum class MonitorMode : std::uint8_t
{
    DIGITAL_ = 0,
    ANALOG_  = 1
};
using VoltageDropCallback = void (*)(void);

class VoltageMonitor
{
public:
    VoltageMonitor(std::uint8_t pin, MonitorMode mode, std::uint16_t threshold, std::uint16_t debounceMs) noexcept;

    bool begin() noexcept;   // configure pin and seed state
    void update() noexcept;  // call from a non‑ISR task periodically

    // Callback on falling event (OK -> not OK)
    void onDrop(VoltageDropCallback cb) noexcept;

    // State / queries (same names)
    [[nodiscard]] std::uint16_t getCurrentLevel() const noexcept
    {
        return _currentLevel;
    }
    [[nodiscard]] bool isVoltageOK() const noexcept
    {
        return _lastVoltageOK;
    }
    bool wasDropDetected() noexcept;  // auto‑clears only via resetDropDetection()
    void resetDropDetection() noexcept
    {
        _dropDetected = false;
    }

    // Configuration (same names)
    void setThreshold(std::uint16_t threshold) noexcept;      // analog threshold (ADC units)
    void setDebounceTime(std::uint16_t debounceMs) noexcept;  // debounce in ms
    void setEnabled(bool en) noexcept
    {
        _enabled = en;
    }
    [[nodiscard]] bool isEnabled() const noexcept
    {
        return _enabled;
    }

    // Optional: set hysteresis for analog mode (0 = disabled). OK if level >= thr+H, not‑OK if <= thr‑H.
    void setHysteresis(std::uint16_t h) noexcept
    {
        _hysteresis = h;
    }

private:
    // Reading helpers
    inline std::uint16_t readVoltageLevel() noexcept;
    inline bool          isVoltageLevelOK(std::uint16_t level) const noexcept;
    inline void          processVoltageChange(std::uint16_t newLevel) noexcept;

private:
    std::uint8_t  _pin;
    MonitorMode   _mode;
    std::uint16_t _threshold;   // ADC threshold or ignored for digital
    std::uint16_t _debounceMs;  // user‑facing (kept for API)
    std::uint32_t _debounceUs;  // internal (us)
    std::uint16_t _hysteresis;  // analog hysteresis (ADC counts)

    bool _enabled;
    bool _initialized;

    std::uint16_t _currentLevel;
    bool          _lastVoltageOK;
    bool          _dropDetected;

    std::uint64_t _lastChangeTimeUs;  // monotonic
    bool          _debouncing;

    VoltageDropCallback _callback;
};

#endif  // VOLTAGE_MONITOR_H

// =============================
// File: VoltageMonitor.cpp
// =============================
#include "VoltageMonitor.h"

VoltageMonitor::VoltageMonitor(std::uint8_t pin, MonitorMode mode, std::uint16_t threshold, std::uint16_t debounceMs) noexcept
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
    VM_LOGD("VoltageMonitor init pin=%u mode=%s thr=%u deb=%u ms", _pin, (_mode == MonitorMode::DIGITAL_) ? "DIGITAL" : "ANALOG", _threshold, _debounceMs);
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
        const std::uint16_t lo = (_threshold > _hysteresis) ? static_cast<std::uint16_t>(_threshold - _hysteresis) : static_cast<std::uint16_t>(0);
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

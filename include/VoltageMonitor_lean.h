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
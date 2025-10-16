#ifndef VOLTAGE_MONITOR_H
#define VOLTAGE_MONITOR_H

#include <Arduino.h>
#include <functional>

/**
 * @brief Voltage level monitoring module for ESP32
 *
 * This module monitors a single GPIO pin for voltage level changes and triggers
 * callbacks when the voltage drops below a threshold. It can be used for power
 * monitoring, battery level detection, or any voltage-based triggering.
 *
 * Features:
 * - Digital or analog voltage monitoring
 * - Configurable threshold levels
 * - Debouncing to prevent false triggers
 * - User-defined callback functions
 * - Lightweight and efficient polling-based monitoring
 */
class VoltageMonitor
{
public:
    // Callback function type for voltage drop events
    using VoltageDropCallback = std::function<void()>;

    // Monitoring modes
    enum class MonitorMode : std::uint8_t
    {
        DIGITAL_ = 0,  // Digital HIGH/LOW monitoring
        ANALOG_  = 1   // Analog threshold-based monitoring
    };

    /**
     * @brief Constructor
     * @param pin GPIO pin to monitor
     * @param mode Monitoring mode (DIGITAL or ANALOG)
     * @param threshold Voltage threshold for triggering (0-4095 for analog, ignored for digital)
     * @param debounceMs Debounce time in milliseconds
     */
    VoltageMonitor(uint8_t pin, MonitorMode mode = MonitorMode::DIGITAL_, uint16_t threshold = 2048, uint16_t debounceMs = 50);

    /**
     * @brief Initialize the voltage monitor
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Register callback function for voltage drop events
     * @param callback Function to call when voltage drop is detected
     */
    void onDrop(VoltageDropCallback callback);

    /**
     * @brief Update the monitor (call this in main loop or task)
     * This method checks the voltage level and triggers callbacks if needed
     */
    void update();

    /**
     * @brief Get current voltage level
     * @return Current voltage reading (0-4095 for analog, 0/1 for digital)
     */
    uint16_t getCurrentLevel() const;

    /**
     * @brief Get voltage state
     * @return true if voltage is above threshold, false if below
     */
    bool isVoltageOK() const;

    /**
     * @brief Check if voltage drop was detected
     * @return true if voltage drop was detected since last check
     */
    bool wasDropDetected();

    /**
     * @brief Reset drop detection flag
     */
    void resetDropDetection();

    /**
     * @brief Set new threshold value
     * @param threshold New threshold value (0-4095 for analog)
     */
    void setThreshold(uint16_t threshold);

    /**
     * @brief Set debounce time
     * @param debounceMs Debounce time in milliseconds
     */
    void setDebounceTime(uint16_t debounceMs);

    /**
     * @brief Enable or disable monitoring
     * @param enabled true to enable, false to disable
     */
    void setEnabled(bool enabled);

    /**
     * @brief Get monitoring status
     * @return true if monitoring is enabled
     */
    bool isEnabled() const;

private:
    uint8_t     _pin;          ///< GPIO pin to monitor
    MonitorMode _mode;         ///< Monitoring mode
    uint16_t    _threshold;    ///< Voltage threshold
    uint16_t    _debounceMs;   ///< Debounce time in milliseconds
    bool        _enabled;      ///< Monitoring enabled flag
    bool        _initialized;  ///< Initialization flag

    uint16_t _currentLevel;    ///< Current voltage level
    bool     _lastVoltageOK;   ///< Last voltage state
    bool     _dropDetected;    ///< Voltage drop detection flag
    uint32_t _lastChangeTime;  ///< Timestamp of last voltage change
    bool     _debouncing;      ///< Debouncing in progress flag

    VoltageDropCallback _callback;  ///< User-defined callback function

    /**
     * @brief Read voltage level from pin
     * @return Voltage reading
     */
    uint16_t readVoltageLevel();

    /**
     * @brief Check if voltage is above threshold
     * @param level Voltage level to check
     * @return true if voltage is OK
     */
    bool isVoltageLevelOK(uint16_t level) const;

    /**
     * @brief Process voltage change
     * @param newLevel New voltage level
     */
    void processVoltageChange(uint16_t newLevel);
};

#endif  // VOLTAGE_MONITOR_H
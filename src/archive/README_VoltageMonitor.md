# VoltageMonitor Module

A lightweight and efficient C++ module for ESP32 that monitors voltage levels on GPIO pins and triggers callbacks when voltage drops are detected.

## Features

- **Dual Monitoring Modes**: Digital (HIGH/LOW) and Analog (threshold-based) monitoring
- **Configurable Thresholds**: Set custom voltage thresholds for analog monitoring
- **Debouncing**: Built-in debouncing to prevent false triggers
- **Callback System**: User-defined callback functions for voltage drop events
- **Lightweight**: Minimal memory footprint and CPU usage
- **ESP32 Optimized**: Designed specifically for ESP32's ADC and GPIO capabilities
- **Modular Design**: Easy to integrate into larger systems like PowerMonitor or BatteryWatcher

## Quick Start

```cpp
#include "VoltageMonitor.h"

// Create a digital voltage monitor on pin 5
VoltageMonitor powerMonitor(5, VoltageMonitor::MonitorMode::DIGITAL, 0, 100);

// Create an analog voltage monitor on pin 36 (ADC1_CH0)
VoltageMonitor batteryMonitor(36, VoltageMonitor::MonitorMode::ANALOG, 3105, 200);

void onPowerDrop() {
    Serial.println("Power lost!");
    // Implement emergency procedures
}

void onBatteryLow() {
    Serial.println("Battery low!");
    // Implement low battery procedures
}

void setup() {
    // Initialize monitors
    powerMonitor.begin();
    batteryMonitor.begin();
    
    // Register callbacks
    powerMonitor.onDrop(onPowerDrop);
    batteryMonitor.onDrop(onBatteryLow);
}

void loop() {
    // Update monitors (call this regularly)
    powerMonitor.update();
    batteryMonitor.update();
    
    delay(10);
}
```

## API Reference

### Constructor

```cpp
VoltageMonitor(uint8_t pin, MonitorMode mode = MonitorMode::DIGITAL, 
               uint16_t threshold = 2048, uint16_t debounceMs = 50)
```

**Parameters:**
- `pin`: GPIO pin number to monitor
- `mode`: Monitoring mode (`DIGITAL` or `ANALOG`)
- `threshold`: Voltage threshold for analog mode (0-4095, ignored for digital)
- `debounceMs`: Debounce time in milliseconds

### Methods

#### Initialization
```cpp
bool begin()
```
Initializes the voltage monitor. Returns `true` if successful.

#### Callback Registration
```cpp
void onDrop(VoltageDropCallback callback)
```
Registers a callback function to be called when voltage drop is detected.

#### Monitoring
```cpp
void update()
```
Updates the voltage monitor. Call this regularly in your main loop or task.

#### Status Queries
```cpp
uint16_t getCurrentLevel() const
```
Returns the current voltage level (0-4095 for analog, 0/1 for digital).

```cpp
bool isVoltageOK() const
```
Returns `true` if voltage is above threshold, `false` if below.

```cpp
bool wasDropDetected()
```
Returns `true` if voltage drop was detected since last check.

```cpp
void resetDropDetection()
```
Resets the drop detection flag.

#### Configuration
```cpp
void setThreshold(uint16_t threshold)
```
Sets a new voltage threshold (0-4095 for analog mode).

```cpp
void setDebounceTime(uint16_t debounceMs)
```
Sets the debounce time in milliseconds.

```cpp
void setEnabled(bool enabled)
```
Enables or disables monitoring.

```cpp
bool isEnabled() const
```
Returns `true` if monitoring is enabled.

## Monitoring Modes

### Digital Mode
- Monitors digital HIGH/LOW states
- Threshold parameter is ignored
- HIGH (1) = Voltage OK, LOW (0) = Voltage Drop
- Suitable for power supply monitoring, switch detection

### Analog Mode
- Monitors analog voltage levels (0-4095)
- Uses configurable threshold
- Values ≥ threshold = Voltage OK, Values < threshold = Voltage Drop
- Suitable for battery monitoring, voltage divider circuits

## Pin Configuration

### Digital Pins
- Any GPIO pin can be used
- Automatically configured as INPUT
- Optional internal pull-up can be enabled

### Analog Pins (ESP32)
- ADC1: GPIO 32-39
- ADC2: GPIO 0, 2, 4, 12-15, 25-27
- 12-bit resolution (0-4095)
- 3.3V reference voltage

## Usage Examples

### 1. Power Supply Monitoring
```cpp
VoltageMonitor powerMonitor(5, VoltageMonitor::MonitorMode::DIGITAL, 0, 100);

void onPowerLost() {
    // Save critical data
    // Disable motors
    // Enter safe mode
}

void setup() {
    powerMonitor.begin();
    powerMonitor.onDrop(onPowerLost);
}
```

### 2. Battery Level Monitoring
```cpp
// 3.3V battery with voltage divider
// 3.0V = ~3727 ADC, 2.5V = ~3105 ADC
VoltageMonitor batteryMonitor(36, VoltageMonitor::MonitorMode::ANALOG, 3105, 200);

void onBatteryLow() {
    // Reduce power consumption
    // Send warning notifications
    // Graceful shutdown
}

void setup() {
    batteryMonitor.begin();
    batteryMonitor.onDrop(onBatteryLow);
}
```

### 3. Multiple Monitors
```cpp
VoltageMonitor monitors[3] = {
    VoltageMonitor(5, VoltageMonitor::MonitorMode::DIGITAL, 0, 50),   // Main power
    VoltageMonitor(6, VoltageMonitor::MonitorMode::DIGITAL, 0, 50),   // Backup power
    VoltageMonitor(36, VoltageMonitor::MonitorMode::ANALOG, 3000, 100) // Battery
};

void setup() {
    for (int i = 0; i < 3; i++) {
        monitors[i].begin();
    }
}

void loop() {
    for (int i = 0; i < 3; i++) {
        monitors[i].update();
    }
}
```

### 4. RTOS Task Integration
```cpp
void voltageMonitoringTask(void* parameter) {
    VoltageMonitor* monitor = static_cast<VoltageMonitor*>(parameter);
    
    while (true) {
        monitor->update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    VoltageMonitor* monitor = new VoltageMonitor(5);
    monitor->begin();
    
    xTaskCreate(voltageMonitoringTask, "VoltageMonitor", 2048, monitor, 1, NULL);
}
```

### 5. Advanced Usage with Dynamic Thresholds
```cpp
class AdvancedVoltageMonitor {
private:
    VoltageMonitor _monitor;
    uint16_t _normalThreshold;
    uint16_t _criticalThreshold;
    bool _criticalMode;

public:
    AdvancedVoltageMonitor(uint8_t pin) 
        : _monitor(pin, VoltageMonitor::MonitorMode::ANALOG, 3000, 100),
          _normalThreshold(3000),
          _criticalThreshold(2500),
          _criticalMode(false)
    {
    }

    void update() {
        _monitor.update();
        
        // Dynamic threshold adjustment
        uint16_t currentLevel = _monitor.getCurrentLevel();
        
        if (currentLevel < _criticalThreshold && !_criticalMode) {
            _criticalMode = true;
            _monitor.setThreshold(_criticalThreshold);
        }
        else if (currentLevel > _normalThreshold && _criticalMode) {
            _criticalMode = false;
            _monitor.setThreshold(_normalThreshold);
        }
    }
};
```

## Integration with Other Modules

### PowerMonitor Class
```cpp
class PowerMonitor {
private:
    VoltageMonitor _mainPower;
    VoltageMonitor _backupPower;
    bool _usingBackup;

public:
    PowerMonitor() 
        : _mainPower(5, VoltageMonitor::MonitorMode::DIGITAL, 0, 50),
          _backupPower(6, VoltageMonitor::MonitorMode::DIGITAL, 0, 50),
          _usingBackup(false)
    {
        _mainPower.onDrop([this]() { this->onMainPowerLost(); });
        _backupPower.onDrop([this]() { this->onBackupPowerLost(); });
    }

    void update() {
        _mainPower.update();
        _backupPower.update();
    }
};
```

### BatteryWatcher Class
```cpp
class BatteryWatcher {
private:
    VoltageMonitor _battery;
    float _voltage;
    uint8_t _percentage;

public:
    BatteryWatcher(uint8_t pin) 
        : _battery(pin, VoltageMonitor::MonitorMode::ANALOG, 3105, 200),
          _voltage(0.0f),
          _percentage(100)
    {
        _battery.onDrop([this]() { this->onBatteryLow(); });
    }

    void update() {
        _battery.update();
        _voltage = (_battery.getCurrentLevel() / 4095.0f) * 3.3f;
        _percentage = (_voltage / 3.3f) * 100;
    }

    float getVoltage() const { return _voltage; }
    uint8_t getPercentage() const { return _percentage; }
};
```

## Performance Considerations

- **Update Frequency**: Call `update()` every 10-50ms for responsive monitoring
- **Debounce Time**: Use 50-200ms debounce for most applications
- **Memory Usage**: ~50 bytes per monitor instance
- **CPU Usage**: Minimal overhead, suitable for real-time applications

## Troubleshooting

### Common Issues

1. **False Triggers**: Increase debounce time or check for electrical noise
2. **No Triggers**: Verify pin configuration and threshold values
3. **Incorrect Readings**: Check voltage divider ratios for analog monitoring
4. **High CPU Usage**: Reduce update frequency or use RTOS tasks

### Debug Tips

```cpp
// Enable debug logging
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Monitor voltage levels
Serial.printf("Voltage: %d, OK: %s\n", 
              monitor.getCurrentLevel(), 
              monitor.isVoltageOK() ? "YES" : "NO");
```

## License

This module is provided as-is for educational and commercial use. 
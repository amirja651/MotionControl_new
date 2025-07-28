#include "VoltageMonitor.h"
#include <esp_log.h>

static const char* TAG = "VoltageMonitorExample";

// Example 1: Digital voltage monitoring (e.g., power supply monitoring)
VoltageMonitor powerMonitor(5, VoltageMonitor::MonitorMode::DIGITAL_, 0, 100);

// Example 2: Analog voltage monitoring (e.g., battery level monitoring)
// Assuming 3.3V battery with voltage divider to 3.3V max
// 3.0V = ~3727 ADC reading, 2.5V = ~3105 ADC reading
VoltageMonitor batteryMonitor(36, VoltageMonitor::MonitorMode::ANALOG, 3105, 200);

// Example 3: Quick voltage drop detection
VoltageMonitor quickMonitor(4, VoltageMonitor::MonitorMode::DIGITAL_, 0, 10);

// Callback functions
void onPowerDrop()
{
    log_e("POWER DROP DETECTED! System power is down!");
    // Implement emergency shutdown procedures
    // - Save critical data
    // - Disable motors
    // - Enter safe mode
}

void onBatteryLow()
{
    log_w("BATTERY LOW! Voltage below threshold");
    // Implement low battery procedures
    // - Reduce power consumption
    // - Send warning notifications
    // - Graceful shutdown
}

void onQuickDrop()
{
    log_w("Quick voltage drop detected - possible glitch");
    // Implement quick response procedures
    // - Log the event
    // - Check system status
}

void setup()
{
    Serial.begin(115200);
    log_i("VoltageMonitor Example Starting...");

    // Initialize power monitor
    if (powerMonitor.begin())
    {
        powerMonitor.onDrop(onPowerDrop);
        log_i("Power monitor initialized on pin 5");
    }

    // Initialize battery monitor
    if (batteryMonitor.begin())
    {
        batteryMonitor.onDrop(onBatteryLow);
        log_i("Battery monitor initialized on pin 36");
    }

    // Initialize quick monitor
    if (quickMonitor.begin())
    {
        quickMonitor.onDrop(onQuickDrop);
        log_i("Quick monitor initialized on pin 4");
    }
}

void loop()
{
    // Update all voltage monitors
    powerMonitor.update();
    batteryMonitor.update();
    quickMonitor.update();

    // Example: Check for drop detection manually
    if (powerMonitor.wasDropDetected())
    {
        log_e("Power drop was detected!");
        powerMonitor.resetDropDetection();
    }

    if (batteryMonitor.wasDropDetected())
    {
        log_w("Battery low was detected!");
        batteryMonitor.resetDropDetection();
    }

    // Example: Get current voltage levels
    static uint32_t lastReport = 0;
    if (millis() - lastReport > 5000)  // Report every 5 seconds
    {
        log_i("Power level: %s (value: %d)", powerMonitor.isVoltageOK() ? "OK" : "LOW", powerMonitor.getCurrentLevel());

        log_i("Battery level: %s (value: %d)", batteryMonitor.isVoltageOK() ? "OK" : "LOW",
              batteryMonitor.getCurrentLevel());

        lastReport = millis();
    }

    delay(10);  // Small delay to prevent excessive CPU usage
}

// Example: Advanced usage with dynamic threshold adjustment
class AdvancedVoltageMonitor
{
private:
    VoltageMonitor _monitor;
    uint16_t       _normalThreshold;
    uint16_t       _criticalThreshold;
    bool           _criticalMode;

public:
    AdvancedVoltageMonitor(uint8_t pin)
        : _monitor(pin, VoltageMonitor::MonitorMode::ANALOG_, 3000, 100),
          _normalThreshold(3000),
          _criticalThreshold(2500),
          _criticalMode(false)
    {
    }

    bool begin()
    {
        if (!_monitor.begin())
            return false;

        _monitor.onDrop([this]() { this->onVoltageDrop(); });

        return true;
    }

    void update()
    {
        _monitor.update();

        // Dynamic threshold adjustment based on current level
        uint16_t currentLevel = _monitor.getCurrentLevel();

        if (currentLevel < _criticalThreshold && !_criticalMode)
        {
            _criticalMode = true;
            _monitor.setThreshold(_criticalThreshold);
            log_w("Switched to critical threshold mode");
        }
        else if (currentLevel > _normalThreshold && _criticalMode)
        {
            _criticalMode = false;
            _monitor.setThreshold(_normalThreshold);
            log_i("Switched back to normal threshold mode");
        }
    }

private:
    void onVoltageDrop()
    {
        if (_criticalMode)
        {
            log_e("CRITICAL VOLTAGE DROP! Immediate shutdown required!");
            // Implement immediate shutdown
        }
        else
        {
            log_w("Normal voltage drop detected");
            // Implement normal low voltage procedures
        }
    }
};

// Example: RTOS task-based monitoring
void voltageMonitoringTask(void* parameter)
{
    VoltageMonitor* monitor = static_cast<VoltageMonitor*>(parameter);

    while (true)
    {
        monitor->update();
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay
    }
}

// Example: Using VoltageMonitor in a class
class PowerManager
{
private:
    VoltageMonitor _mainPower;
    VoltageMonitor _backupPower;
    bool           _usingBackup;

public:
    PowerManager()
        : _mainPower(5, VoltageMonitor::MonitorMode::DIGITAL_, 0, 50),
          _backupPower(6, VoltageMonitor::MonitorMode::DIGITAL_, 0, 50),
          _usingBackup(false)
    {
        _mainPower.onDrop([this]() { this->onMainPowerLost(); });
        _backupPower.onDrop([this]() { this->onBackupPowerLost(); });
    }

    bool begin()
    {
        bool success = _mainPower.begin() && _backupPower.begin();
        if (success)
        {
            log_i("PowerManager initialized");
        }
        return success;
    }

    void update()
    {
        _mainPower.update();
        _backupPower.update();
    }

    bool isUsingBackupPower() const
    {
        return _usingBackup;
    }

private:
    void onMainPowerLost()
    {
        log_w("Main power lost, switching to backup");
        _usingBackup = true;
        // Implement backup power activation
    }

    void onBackupPowerLost()
    {
        log_e("Backup power also lost! System shutdown!");
        _usingBackup = false;
        // Implement emergency shutdown
    }
};
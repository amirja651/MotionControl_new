#include "SystemDiagnostics.h"

void SystemDiagnostics::initialize()
{
    // No initialization needed
}

void SystemDiagnostics::printSystemInfo()
{
    Serial.println("\n=== System Information ===");
    printChipInfo();
    printMemoryInfo();
    Serial.println("========================\n");
}

void SystemDiagnostics::printSystemStatus()
{
    Serial.println("\n=== System Status ===");
    printResetReason();
    printTaskInfo();
    Serial.println("====================\n");
}

void SystemDiagnostics::printChipInfo()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    Serial.printf("Chip Model: %s\n", getChipModel());
    Serial.printf("Chip Revision: %d\n", chip_info.revision);
    Serial.printf("CPU Cores: %d\n", chip_info.cores);
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("Flash Speed: %d MHz\n", ESP.getFlashChipSpeed() / 1000000);
    Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
}

void SystemDiagnostics::printMemoryInfo()
{
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
    Serial.printf("Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());
    Serial.printf("Max Alloc Heap: %d bytes\n", ESP.getMaxAllocHeap());
}

void SystemDiagnostics::printResetReason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("Reset Reason: %s\n", getResetReason(reason));
    Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
}

void SystemDiagnostics::printTaskInfo()
{
    Serial.printf("Free Stack Space: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));
    Serial.printf("Task Count: %d\n", uxTaskGetNumberOfTasks());
}

const char* SystemDiagnostics::getResetReason(esp_reset_reason_t reason)
{
    switch (reason)
    {
        case ESP_RST_POWERON:
            return "Power On";
        case ESP_RST_EXT:
            return "External Reset";
        case ESP_RST_SW:
            return "Software Reset";
        case ESP_RST_PANIC:
            return "Panic Reset";
        case ESP_RST_INT_WDT:
            return "Interrupt Watchdog";
        case ESP_RST_TASK_WDT:
            return "Task Watchdog";
        case ESP_RST_WDT:
            return "Watchdog";
        case ESP_RST_DEEPSLEEP:
            return "Deep Sleep";
        case ESP_RST_BROWNOUT:
            return "Brownout";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "Unknown";
    }
}

const char* SystemDiagnostics::getChipModel()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    switch (chip_info.model)
    {
        case CHIP_ESP32:
            return "ESP32";
        case CHIP_ESP32S2:
            return "ESP32-S2";
        case CHIP_ESP32S3:
            return "ESP32-S3";
        case CHIP_ESP32C3:
            return "ESP32-C3";
        case CHIP_ESP32H2:
            return "ESP32-H2";
        default:
            return "Unknown";
    }
}
#include "SystemDiagnostics.h"

void SystemDiagnostics::printSystemInfo()
{
    Serial.println();
    printChipInfo();
    printMemoryInfo();
}

void SystemDiagnostics::printSystemStatus()
{
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.println();
    Serial.printf("┌─────────────┬─────────────┬─────────────┬─────────────┐\n");
    Serial.printf("│ Reset       │ Uptime      │ Free Stack  │ Task        │\n");
    Serial.printf("│ Reason      │ (seconds)   │ Space       │ Count       │\n");
    Serial.printf("├─────────────┼─────────────┼─────────────┼─────────────┤\n");
    Serial.printf("│ %-11s │ %-11lu │ %-11d │ %-11d │\n", getResetReason(reason), millis() / 1000, uxTaskGetStackHighWaterMark(NULL), uxTaskGetNumberOfTasks());
    Serial.printf("└─────────────┴─────────────┴─────────────┴─────────────┘\n");
}

void SystemDiagnostics::printChipInfo()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.println();
    Serial.printf("┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐\n");
    Serial.printf("│ Chip Model  │ Chip Rev.   │ CPU Cores   │ CPU Freq.   │ Flash Size  │ Flash Speed │ SDK Version │\n");
    Serial.printf("├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤\n");
    Serial.printf("│ %-11s │ %-11d │ %-11d │ %-11d │ %-11d │ %-11d │ %-11s │\n", getChipModel(), chip_info.revision, chip_info.cores, ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / (1024 * 1024), ESP.getFlashChipSpeed() / 1000000, ESP.getSdkVersion());
    Serial.printf("└─────────────┴─────────────┴─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘\n");
}

void SystemDiagnostics::printMemoryInfo()
{
    Serial.println();
    Serial.printf("┌─────────────┬─────────────┬─────────────┬──────────────┐\n");
    Serial.printf("│ Free Heap   │ Free PSRAM  │ Min Free    │ Max Alloc    │\n");
    Serial.printf("│             │             │ Heap        │ Heap         │\n");
    Serial.printf("├─────────────┼─────────────┼─────────────┼──────────────┤\n");
    Serial.printf("│ %-11d │ %-11d │ %-11d │ %-12d │\n", ESP.getFreeHeap(), ESP.getFreePsram(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
    Serial.printf("└─────────────┴─────────────┴─────────────┴──────────────┘\n");
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
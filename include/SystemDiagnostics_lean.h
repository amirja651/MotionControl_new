// =============================
// File: SystemDiagnostics.h
// Lean, allocation‑free, RT‑friendly diagnostics for ESP32 (Arduino)
// Keeps printing out of RT paths; supports pretty/compact/CSV outputs.
// Backward‑compatible wrappers included.
// =============================
#ifndef SYSTEM_DIAGNOSTICS_H
    #define SYSTEM_DIAGNOSTICS_H

    #include <Arduino.h>
    #include <cstdint>
    #include <esp_chip_info.h>
    #include <esp_system.h>
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>

    // ---- Compile‑time switches
    #ifndef DIAG_ENABLE
        #define DIAG_ENABLE 1  // 0: disable all printing helpers
    #endif

    #ifndef DIAG_BOX_DRAWING
        #define DIAG_BOX_DRAWING 1  // 1: unicode box characters; 0: ASCII
    #endif

struct SysSnapshot
{
    // Time
    uint64_t uptime_us{0};
    uint32_t uptime_s{0};

    // Chip / SDK
    const char* chip_model_str{"Unknown"};
    uint8_t     chip_revision{0};
    uint8_t     cpu_cores{0};
    uint32_t    cpu_freq_mhz{0};
    uint32_t    flash_size_mb{0};
    uint32_t    flash_speed_mhz{0};
    const char* sdk_version{""};

    // Memory
    uint32_t free_heap{0};
    uint32_t min_free_heap{0};
    uint32_t max_alloc_heap{0};
    uint32_t free_psram{0};

    // RTOS / reset
    const char*        reset_reason_str{"Unknown"};
    esp_reset_reason_t reset_reason{ESP_RST_UNKNOWN};
    uint32_t           task_count{0};
    uint32_t           stack_high_water_bytes{0};  // for provided task (current if null)
};

class SystemDiagnostics
{
public:
    // Collect a snapshot (no prints). If task==nullptr -> current task.
    static SysSnapshot read(TaskHandle_t task = nullptr) noexcept;

    // Pretty table output (Unicode boxes by default). Safe to call from a low‑prio logger task.
    static void printPretty(const SysSnapshot& s) noexcept;
    // One‑liner compact output (good for logs).
    static void printCompact(const SysSnapshot& s) noexcept;
    // CSV helpers for logging to file/serial plotter.
    static void printCsvHeader() noexcept;
    static void printCsvRow(const SysSnapshot& s) noexcept;

    // ---- Backward‑compatible wrappers with your original names ----
    static void printSystemInfo() noexcept;    // chip + memory
    static void printSystemStatus() noexcept;  // reset + uptime + stack + tasks

    // Small helpers
    static const char* getResetReasonStr(esp_reset_reason_t r) noexcept;
    static const char* getChipModelStr() noexcept;
};

#endif  // SYSTEM_DIAGNOSTICS_H

// =============================
// File: SystemDiagnostics.cpp
// =============================
#include "SystemDiagnostics.h"

static inline uint32_t get_stack_high_water_bytes(TaskHandle_t t) noexcept
{
    // uxTaskGetStackHighWaterMark returns words; convert to bytes.
#if (portSTACK_GROWTH < 0)
    // Typical for ESP32/FreeRTOS
    const UBaseType_t words = uxTaskGetStackHighWaterMark(t);
    return static_cast<uint32_t>(words) * sizeof(StackType_t);
#else
    return static_cast<uint32_t>(uxTaskGetStackHighWaterMark(t)) * sizeof(StackType_t);
#endif
}

static inline const char* chip_model_from_info(const esp_chip_info_t& info) noexcept
{
    switch (info.model)
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

const char* SystemDiagnostics::getChipModelStr() noexcept
{
    esp_chip_info_t info{};
    esp_chip_info(&info);
    return chip_model_from_info(info);
}

const char* SystemDiagnostics::getResetReasonStr(esp_reset_reason_t r) noexcept
{
    switch (r)
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

SysSnapshot SystemDiagnostics::read(TaskHandle_t task) noexcept
{
    SysSnapshot s{};

    // Time (use esp_timer to avoid millis() rollover ~49.7d)
    const uint64_t us = static_cast<uint64_t>(esp_timer_get_time());
    s.uptime_us       = us;
    s.uptime_s        = static_cast<uint32_t>(us / 1000000ULL);

    // Chip / SDK
    esp_chip_info_t info{};
    esp_chip_info(&info);
    s.chip_model_str  = chip_model_from_info(info);
    s.chip_revision   = static_cast<uint8_t>(info.revision);
    s.cpu_cores       = static_cast<uint8_t>(info.cores);
    s.cpu_freq_mhz    = static_cast<uint32_t>(ESP.getCpuFreqMHz());
    s.flash_size_mb   = static_cast<uint32_t>(ESP.getFlashChipSize() / (1024 * 1024));
    s.flash_speed_mhz = static_cast<uint32_t>(ESP.getFlashChipSpeed() / 1000000);
    s.sdk_version     = ESP.getSdkVersion();

    // Memory
    s.free_heap      = static_cast<uint32_t>(ESP.getFreeHeap());
    s.min_free_heap  = static_cast<uint32_t>(ESP.getMinFreeHeap());
    s.max_alloc_heap = static_cast<uint32_t>(ESP.getMaxAllocHeap());
    s.free_psram     = static_cast<uint32_t>(ESP.getFreePsram());

    // RTOS / reset
    s.reset_reason           = esp_reset_reason();
    s.reset_reason_str       = getResetReasonStr(s.reset_reason);
    s.task_count             = static_cast<uint32_t>(uxTaskGetNumberOfTasks());
    s.stack_high_water_bytes = get_stack_high_water_bytes(task);

    return s;
}

#if DIAG_ENABLE

void SystemDiagnostics::printPretty(const SysSnapshot& s) noexcept
{
    #if DIAG_BOX_DRAWING
    Serial.println();
    Serial.printf("┌─────────────┬─────────────┬─────────────┬─────────────┐\n");
    Serial.printf("│ Reset       │ Uptime      │ Free Stack  │ Task        │\n");
    Serial.printf("│ Reason      │ (seconds)   │ HighWater   │ Count       │\n");
    Serial.printf("├─────────────┼─────────────┼─────────────┼─────────────┤\n");
    Serial.printf("│ %-11s │ %-11lu │ %-11u │ %-11u │\n", s.reset_reason_str, static_cast<unsigned long>(s.uptime_s), static_cast<unsigned>(s.stack_high_water_bytes), static_cast<unsigned>(s.task_count));
    Serial.printf("└─────────────┴─────────────┴─────────────┴─────────────┘\n");

    Serial.println();
    Serial.printf("┌─────────────┬─────────────┬─────────────┬──────────────┐\n");
    Serial.printf("│ Free Heap   │ Free PSRAM  │ Min Free    │ Max Alloc    │\n");
    Serial.printf("│             │             │ Heap        │ Heap         │\n");
    Serial.printf("├─────────────┼─────────────┼─────────────┼──────────────┤\n");
    Serial.printf("│ %-11u │ %-11u │ %-11u │ %-12u │\n", s.free_heap, s.free_psram, s.min_free_heap, s.max_alloc_heap);
    Serial.printf("└─────────────┴─────────────┴─────────────┴──────────────┘\n");

    Serial.println();
    Serial.printf("┌─────────────┬─────────────┬─────────────┬─────────────┬─────────────┬─────────────┬─────────────┐\n");
    Serial.printf("│ Chip Model  │ Chip Rev.   │ CPU Cores   │ CPU Freq.   │ Flash Size  │ Flash Speed │ SDK Version │\n");
    Serial.printf("├─────────────┼─────────────┼─────────────┼─────────────┼─────────────┼─────────────┼─────────────┤\n");
    Serial.printf("│ %-11s │ %-11u │ %-11u │ %-11u │ %-11u │ %-11u │ %-11s │\n", s.chip_model_str, static_cast<unsigned>(s.chip_revision), static_cast<unsigned>(s.cpu_cores), s.cpu_freq_mhz, s.flash_size_mb, s.flash_speed_mhz, s.sdk_version);
    Serial.printf("└─────────────┴─────────────┴─────────────┴─────────────┴─────────────┴─────────────┴─────────────┘\n");
    #else
    // ASCII variant
    Serial.println();
    Serial.printf("+------------+------------+------------+------------+\n");
    Serial.printf("| Reset      | Uptime(s)  | StackHW(B) | Tasks      |\n");
    Serial.printf("+------------+------------+------------+------------+\n");
    Serial.printf("| %-10s | %-10lu | %-10u | %-10u |\n", s.reset_reason_str, static_cast<unsigned long>(s.uptime_s), static_cast<unsigned>(s.stack_high_water_bytes), static_cast<unsigned>(s.task_count));
    Serial.printf("+------------+------------+------------+------------+\n");

    Serial.println();
    Serial.printf("+------------+------------+------------+-------------+\n");
    Serial.printf("| FreeHeap   | FreePSRAM  | MinFree    | MaxAlloc    |\n");
    Serial.printf("+------------+------------+------------+-------------+\n");
    Serial.printf("| %-10u | %-10u | %-10u | %-11u |\n", s.free_heap, s.free_psram, s.min_free_heap, s.max_alloc_heap);
    Serial.printf("+------------+------------+------------+-------------+\n");

    Serial.println();
    Serial.printf("+------------+------------+------------+------------+------------+------------+------------+\n");
    Serial.printf("| ChipModel  | ChipRev    | Cores      | CPU(MHz)   | Flash(MB)  | Flash(MHz) | SDK        |\n");
    Serial.printf("+------------+------------+------------+------------+------------+------------+------------+\n");
    Serial.printf("| %-10s | %-10u | %-10u | %-10u | %-10u | %-10u | %-10s |\n", s.chip_model_str, static_cast<unsigned>(s.chip_revision), static_cast<unsigned>(s.cpu_cores), s.cpu_freq_mhz, s.flash_size_mb, s.flash_speed_mhz, s.sdk_version);
    Serial.printf("+------------+------------+------------+------------+------------+------------+------------+\n");
    #endif
}

void SystemDiagnostics::printCompact(const SysSnapshot& s) noexcept
{
    log_d("\n\n--- reset = %s \n--- uptime_s = %lu \n--- stackHW_B = %u \n--- tasks = %u \n--- freeHeap = %u \n--- freePSRAM = %u \n--- minHeap = %u \n--- maxAlloc = %u \n--- cpu = %uMHz \n--- flash = %uMB @ %uMHz \n--- sdk = %s\n",
          s.reset_reason_str,
          static_cast<unsigned long>(s.uptime_s),
          static_cast<unsigned>(s.stack_high_water_bytes),
          static_cast<unsigned>(s.task_count),
          s.free_heap,
          s.free_psram,
          s.min_free_heap,
          s.max_alloc_heap,
          s.cpu_freq_mhz,
          s.flash_size_mb,
          s.flash_speed_mhz,
          s.sdk_version);
}

void SystemDiagnostics::printCsvHeader() noexcept
{
    Serial.println("uptime_s,reset_reason,stackHW_B,tasks,freeHeap,freePSRAM,minHeap,maxAlloc,cpu_MHz,flash_MB,flash_MHz,sdk,chip,rev,cores");
}

void SystemDiagnostics::printCsvRow(const SysSnapshot& s) noexcept
{
    Serial.printf("%lu,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%s,%s,%u,%u\n",
                  static_cast<unsigned long>(s.uptime_s),
                  s.reset_reason_str,
                  static_cast<unsigned>(s.stack_high_water_bytes),
                  static_cast<unsigned>(s.task_count),
                  s.free_heap,
                  s.free_psram,
                  s.min_free_heap,
                  s.max_alloc_heap,
                  s.cpu_freq_mhz,
                  s.flash_size_mb,
                  s.flash_speed_mhz,
                  s.sdk_version,
                  s.chip_model_str,
                  static_cast<unsigned>(s.chip_revision),
                  static_cast<unsigned>(s.cpu_cores));
}

void SystemDiagnostics::printSystemInfo() noexcept
{
    const SysSnapshot s = read();
    // Reuse pretty printing but only chip/memory sections
    printPretty(s);
}

void SystemDiagnostics::printSystemStatus() noexcept
{
    const SysSnapshot s = read();
    // Show compact status line (fast) + one memory table if needed
    printCompact(s);
}

#else  // DIAG_ENABLE == 0

void SystemDiagnostics::printPretty(const SysSnapshot&) noexcept {}
void SystemDiagnostics::printCompact(const SysSnapshot&) noexcept {}
void SystemDiagnostics::printCsvHeader() noexcept {}
void SystemDiagnostics::printCsvRow(const SysSnapshot&) noexcept {}
void SystemDiagnostics::printSystemInfo() noexcept {}
void SystemDiagnostics::printSystemStatus() noexcept {}

#endif  // DIAG_ENABLE

#ifndef SYSTEM_DIAGNOSTICS_H
#define SYSTEM_DIAGNOSTICS_H

#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_rom_sys.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

class SystemDiagnostics
{
public:
    static void initialize();
    static void printSystemInfo();
    static void printSystemStatus();

private:
    static void printChipInfo();
    static void printMemoryInfo();
    static void printResetReason();
    static void printTaskInfo();

    static const char* getResetReason(esp_reset_reason_t reason);
    static const char* getChipModel();
};

#endif  // SYSTEM_DIAGNOSTICS_H
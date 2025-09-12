#include "Pins.h"
#include "TMC5160Manager_lean.h"
#include "esp_log.h"
#include <Arduino.h>
#include <SPI.h>

static constexpr std::int32_t SPI_CLOCK = 1000000;  // 1MHz SPI clock

static bool    driverEnabled[4] = {false, false, false, false};
TMC5160Manager driver[4]        = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

void setup()
{
    SPI.begin(SPIPins::SCK, SPIPins::MISO, SPIPins::MOSI);
    SPI.setFrequency(SPI_CLOCK);
    SPI.setDataMode(SPI_MODE3);
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_INFO);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    //------------------------------------------------
    const char* txt = "**NOT**";
#ifdef CONFIG_FREERTOS_CHECK_STACKOVERFLOW
    txt = "";
#endif
    log_d("Stack overflow checking %s enabled", txt);
    //------------------------------------------------

    // for disable all drivers pins - for avoid conflict in SPI bus
    // Initialize CS pins and turn them off with safety checks
    for (std::size_t i = 0U; i < 4; ++i)
    {
        pinMode(DriverPins::CS[i], OUTPUT);
        digitalWrite(DriverPins::CS[i], HIGH);
    }

    for (std::size_t i = 0U; i < 4; ++i)
    {
        // Initialize TMC5160 drivers
        log_d("TMC5160 init:");
        if (!driver[i].begin())
        {
            log_e("--- Driver[%d] init ❌ failed", i);
        }
        else
        {
            driver[i].configureDriver_All_Motors(true);  // true = StealthChop
            driver[i].setRmsCurrent(350);                // mA
            driver[i].setIrun(16);
            driver[i].setIhold(8);
            driver[i].setMicrosteps(32);

            // Optional: quick sanity check
            if (!driver[i].testConnection(true))
            {
                log_e("--- SPI/driver[%d] ⚠️ not responding", i);
            }
            else
            {
                log_d("--- SPI/driver[%d] is 👍 OK", i);
            }
        }
    }
}

void loop()
{
    delay(10);
}
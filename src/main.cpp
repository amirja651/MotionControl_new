#include "DirMultiplexer-lean.h"
#include "Pins.h"
#include "SystemDiagnostics_lean.h"
#include "TMC5160Manager_lean.h"
#include "UnitConverter-lean.h"
#include "esp_log.h"
#include <Arduino.h>
#include <SPI.h>

static constexpr std::int32_t SPI_CLOCK = 1000000;  // 1MHz SPI clock

static bool    driverEnabled[4] = {false, false, false, false};
TMC5160Manager driver[4]        = {TMC5160Manager(0, DriverPins::CS[0]), TMC5160Manager(1, DriverPins::CS[1]), TMC5160Manager(2, DriverPins::CS[2]), TMC5160Manager(3, DriverPins::CS[3])};

DirMultiplexer dirmux(MultiplexerPins::S0, MultiplexerPins::S1, MultiplexerPins::DIR);

static void printConversionTable(const ConvertValues& cvfd, const ConvertValues& cvfp, const ConvertValues& cvfs, const ConvertValues& cvfm);

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

    auto s = SystemDiagnostics::read();  // Collection only
    SystemDiagnostics::printCompact(s);  // Fast one-liner

    // Or:
    // SystemDiagnostics::printPretty(s); // Full table
    // SystemDiagnostics::printCsvHeader(); // Once in setup
    // SystemDiagnostics::printCsvRow(s); // For logging

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

    //------------------------------------------------

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

    //------------------------------------------------

    (void)dirmux.begin();  // Safe mode: Motor0, DIR=LOW
                           // dirmux.setMotorDirection(2, 1);  // Select Motor2 and DIR=HIGH
                           // dirmux.selectMotor(3); // Quickly switch between motors

    //------------------------------------------------

    // Typical defaults (adjust if yours differ)
    UnitConverter::setDefaultResolution(4096);              // pulses per rev
    UnitConverter::setDefaultMicrometers(200.0);            // µm per rev (0.2 mm lead)
    UnitConverter::setDefaultMicrosteps(200 * 64);          // 200 steps * 64 µsteps = 12800
    UnitConverter::setDefaultMotorType(MotorType::LINEAR);  // To see the value of TURNS (if ROTATIONAL, TURNS=0)

    auto cvfd = UnitConverter::convertFromDegrees(1.0);
    auto cvfp = UnitConverter::convertFromPulses(1.0);
    auto cvfs = UnitConverter::convertFromSteps(1);
    auto cvfm = UnitConverter::convertFromMicrometers(1.0);

    Serial.println();
    printConversionTable(cvfd, cvfp, cvfs, cvfm);
    Serial.println();

    //------------------------------------------------
}

void loop()
{
    delay(10);
}

// Pretty-print a fixed-width ASCII table of conversions
static void printConversionTable(const ConvertValues& cvfd, const ConvertValues& cvfp, const ConvertValues& cvfs, const ConvertValues& cvfm)
{
    auto line = []()
    {
        Serial.println("+----------------------+------------+------------+------------+-------"
                       "-----+--------------+");
    };
    line();
    Serial.printf("| %-20s | %10s | %10s | %10s | %10s | %12s |\n", "From (unit)", "Degrees", "Pulses", "Steps", "Turns", "Micrometers");
    line();

    // From Degrees (inputValue interpreted as degrees)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Degrees", cvfd.TO_DEGREES, static_cast<double>(cvfd.TO_PULSES), static_cast<long>(cvfd.TO_STEPS), static_cast<long>(cvfd.TO_TURNS), static_cast<double>(cvfd.TO_MICROMETERS));

    // From Pulses (inputValue interpreted as pulses)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Pulses", static_cast<double>(cvfp.TO_DEGREES), static_cast<double>(cvfp.TO_PULSES), static_cast<long>(cvfp.TO_STEPS), static_cast<long>(cvfp.TO_TURNS), static_cast<double>(cvfp.TO_MICROMETERS));

    // From Steps (inputValue interpreted as steps)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Steps", static_cast<double>(cvfs.TO_DEGREES), static_cast<double>(cvfs.TO_PULSES), static_cast<long>(cvfs.TO_STEPS), static_cast<long>(cvfs.TO_TURNS), static_cast<double>(cvfs.TO_MICROMETERS));

    // From Micrometers (inputValue interpreted as µm)
    Serial.printf("| %-20s | %10.2f | %10.2f | %10ld | %10ld | %12.3f |\n", "Micrometers", static_cast<double>(cvfm.TO_DEGREES), static_cast<double>(cvfm.TO_PULSES), static_cast<long>(cvfm.TO_STEPS), static_cast<long>(cvfm.TO_TURNS), static_cast<double>(cvfm.TO_MICROMETERS));

    line();
}

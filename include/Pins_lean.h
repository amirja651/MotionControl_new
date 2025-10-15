// =============================
// File: Pins.h  (header-only, API-compatible)
// Lean, allocation-free, RT-friendly pin map for ESP32 (Arduino)
// - 100% compatible with your existing Pins.h names (DriverPins, SPIPins, EncoderPins, MultiplexerPins,
// VoltageMonitorPins)
// - Replaces need for Pins.cpp by using C++17 inline constexprs (no linker errors)
// - Adds small helpers and compile-time sanity checks
// =============================
#ifndef PINS_H
#define PINS_H

#include <cstddef>
#include <cstdint>

// ----- Channel count (kept at 4 as in your code)
inline constexpr std::size_t PINS_CHANNELS = 4;

// ----- Same structs & members as your original header -----
struct DriverPins
{
    // Use uint16_t as in your file for full GPIO range compatibility
    static inline constexpr std::uint16_t DIR[PINS_CHANNELS]  = {22, 4, 32, 27};
    static inline constexpr std::uint16_t STEP[PINS_CHANNELS] = {21, 16, 33, 14};
    static inline constexpr std::uint16_t EN[PINS_CHANNELS]   = {17, 15, 26, 13};
    static inline constexpr std::uint16_t CS[PINS_CHANNELS]   = {5, 2, 25, 12};
};

struct SPIPins
{
    static inline constexpr std::uint16_t MOSI = 23;
    static inline constexpr std::uint16_t MISO = 19;
    static inline constexpr std::uint16_t SCK  = 18;
};

struct EncoderPins
{
    static inline constexpr std::uint16_t SIGNAL[PINS_CHANNELS] = {36, 39, 34, 35};
};

struct MultiplexerPins
{
    // Kept exactly as your original public names
    static inline constexpr std::uint16_t S0  = 22;  // Select line 0
    static inline constexpr std::uint16_t S1  = 4;   // Select line 1
    static inline constexpr std::uint16_t DIR = 32;  // Direction signal (common Z)
};

struct VoltageMonitorPins
{
    static inline constexpr std::uint16_t POWER_3_3 = 27;
};

// ----- Optional typed channel id for readability -----
enum class Chan : std::uint8_t
{
    M0 = 0,
    M1 = 1,
    M2 = 2,
    M3 = 3
};

// ----- Tiny helpers (constexpr; do NOT change original API) -----
namespace pins_helpers
{
    // Per-channel driver & encoder pins
    constexpr std::uint16_t dir(std::uint8_t ch) noexcept
    {
        return DriverPins::DIR[ch];
    }
    constexpr std::uint16_t step(std::uint8_t ch) noexcept
    {
        return DriverPins::STEP[ch];
    }
    constexpr std::uint16_t en(std::uint8_t ch) noexcept
    {
        return DriverPins::EN[ch];
    }
    constexpr std::uint16_t cs(std::uint8_t ch) noexcept
    {
        return DriverPins::CS[ch];
    }
    constexpr std::uint16_t enc(std::uint8_t ch) noexcept
    {
        return EncoderPins::SIGNAL[ch];
    }

    constexpr std::uint16_t dir(Chan ch) noexcept
    {
        return dir(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t step(Chan ch) noexcept
    {
        return step(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t en(Chan ch) noexcept
    {
        return en(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t cs(Chan ch) noexcept
    {
        return cs(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t enc(Chan ch) noexcept
    {
        return enc(static_cast<std::uint8_t>(ch));
    }

    // Multiplexer helpers (singletons)
    constexpr std::uint16_t mux_s0() noexcept
    {
        return MultiplexerPins::S0;
    }
    constexpr std::uint16_t mux_s1() noexcept
    {
        return MultiplexerPins::S1;
    }
    constexpr std::uint16_t mux_dir() noexcept
    {
        return MultiplexerPins::DIR;
    }

    // Voltage monitor helper (3.3V rail sense pin)
    constexpr std::uint16_t power_3v3() noexcept
    {
        return VoltageMonitorPins::POWER_3_3;
    }

    // SPI bus helpers (singletons)
    constexpr std::uint16_t spi_mosi() noexcept
    {
        return SPIPins::MOSI;
    }
    constexpr std::uint16_t spi_miso() noexcept
    {
        return SPIPins::MISO;
    }
    constexpr std::uint16_t spi_sck() noexcept
    {
        return SPIPins::SCK;
    }
}  // namespace pins_helpers

// ----- Compile-time sanity checks for classic ESP32 (optional, can be removed) -----
constexpr bool _all_lt_34(const std::uint16_t* a, std::size_t n)
{
    for (std::size_t i = 0; i < n; ++i)
        if (a[i] >= 34)
            return false;
    return true;
}
static_assert(_all_lt_34(DriverPins::DIR, PINS_CHANNELS), "DIR pins must be output-capable (<34)");
static_assert(_all_lt_34(DriverPins::STEP, PINS_CHANNELS), "STEP pins must be output-capable (<34)");
static_assert(_all_lt_34(DriverPins::EN, PINS_CHANNELS), "EN pins must be output-capable (<34)");
static_assert(_all_lt_34(DriverPins::CS, PINS_CHANNELS), "CS pins must be output-capable (<34)");
static_assert(MultiplexerPins::S0 < 34 && MultiplexerPins::S1 < 34 && MultiplexerPins::DIR < 34, "Multiplexer S0/S1/DIR must be output-capable (<34)");
// Encoder pins (34..39) are input-only by design → OK.

#endif  // PINS_H

/*
Usage (API-compatible):
  #include "Pins.h"
  pinMode(DriverPins::DIR[0], OUTPUT);
  pinMode(EncoderPins::SIGNAL[1], INPUT);
  digitalWrite(DriverPins::EN[2], HIGH);

Optional helpers (readability):
  using namespace pins_helpers;
  pinMode(dir(Chan::M0), OUTPUT);
  pinMode(step(1), OUTPUT);
  auto sck  = SPIPins::SCK;
  auto s0   = mux_s0();
  auto s1   = mux_s1();
  auto md   = mux_dir();
  auto p3v3 = power_3v3();
*/
// =============================
// File: Pins.h  (header-only, API-compatible)
// Lean, allocation-free, RT-friendly pin map for ESP32 (Arduino)
// - 100% compatible with your existing Pins.h names (DriverPins, SPIPins, EncoderPins, MultiplexerPins,
// VoltageMonitorPins)
// - Replaces need for Pins.cpp by using C++17 inline constexprs (no linker errors)
// - Adds small helpers and compile-time sanity checks
// =============================
#ifndef PINS_H
#define PINS_H

#include <cstddef>
#include <cstdint>

// ----- Channel count (kept at 4 as in your code)
inline constexpr std::size_t PINS_CHANNELS = 4;

// ----- Same structs & members as your original header -----
struct DriverPins
{
    // Use uint16_t as in your file for full GPIO range compatibility
    static inline constexpr std::uint16_t DIR[PINS_CHANNELS]  = {22, 4, 32, 27};
    static inline constexpr std::uint16_t STEP[PINS_CHANNELS] = {21, 16, 33, 14};
    static inline constexpr std::uint16_t EN[PINS_CHANNELS]   = {17, 15, 26, 13};
    static inline constexpr std::uint16_t CS[PINS_CHANNELS]   = {5, 2, 25, 12};
};

struct SPIPins
{
    static inline constexpr std::uint16_t MOSI = 23;
    static inline constexpr std::uint16_t MISO = 19;
    static inline constexpr std::uint16_t SCK  = 18;
};

struct EncoderPins
{
    static inline constexpr std::uint16_t SIGNAL[PINS_CHANNELS] = {36, 39, 34, 35};
};

struct MultiplexerPins
{
    // Kept exactly as your original public names
    static inline constexpr std::uint16_t S0  = 22;  // Select line 0
    static inline constexpr std::uint16_t S1  = 4;   // Select line 1
    static inline constexpr std::uint16_t DIR = 32;  // Direction signal (common Z)
};

struct VoltageMonitorPins
{
    static inline constexpr std::uint16_t POWER_3_3 = 27;
};

// ----- Optional typed channel id for readability -----
enum class Chan : std::uint8_t
{
    M0 = 0,
    M1 = 1,
    M2 = 2,
    M3 = 3
};

// ----- Tiny helpers (constexpr; do NOT change original API) -----
namespace pins_helpers
{
    // Per-channel driver & encoder pins
    constexpr std::uint16_t dir(std::uint8_t ch) noexcept
    {
        return DriverPins::DIR[ch];
    }
    constexpr std::uint16_t step(std::uint8_t ch) noexcept
    {
        return DriverPins::STEP[ch];
    }
    constexpr std::uint16_t en(std::uint8_t ch) noexcept
    {
        return DriverPins::EN[ch];
    }
    constexpr std::uint16_t cs(std::uint8_t ch) noexcept
    {
        return DriverPins::CS[ch];
    }
    constexpr std::uint16_t enc(std::uint8_t ch) noexcept
    {
        return EncoderPins::SIGNAL[ch];
    }

    constexpr std::uint16_t dir(Chan ch) noexcept
    {
        return dir(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t step(Chan ch) noexcept
    {
        return step(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t en(Chan ch) noexcept
    {
        return en(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t cs(Chan ch) noexcept
    {
        return cs(static_cast<std::uint8_t>(ch));
    }
    constexpr std::uint16_t enc(Chan ch) noexcept
    {
        return enc(static_cast<std::uint8_t>(ch));
    }

    // Multiplexer helpers (singletons)
    constexpr std::uint16_t mux_s0() noexcept
    {
        return MultiplexerPins::S0;
    }
    constexpr std::uint16_t mux_s1() noexcept
    {
        return MultiplexerPins::S1;
    }
    constexpr std::uint16_t mux_dir() noexcept
    {
        return MultiplexerPins::DIR;
    }

    // Voltage monitor helper (3.3V rail sense pin)
    constexpr std::uint16_t power_3v3() noexcept
    {
        return VoltageMonitorPins::POWER_3_3;
    }
}  // namespace pins_helpers

// ----- Compile-time sanity checks for classic ESP32 (optional, can be removed) -----
constexpr bool _all_lt_34(const std::uint16_t* a, std::size_t n)
{
    for (std::size_t i = 0; i < n; ++i)
        if (a[i] >= 34)
            return false;
    return true;
}
static_assert(_all_lt_34(DriverPins::DIR, PINS_CHANNELS), "DIR pins must be output-capable (<34)");
static_assert(_all_lt_34(DriverPins::STEP, PINS_CHANNELS), "STEP pins must be output-capable (<34)");
static_assert(_all_lt_34(DriverPins::EN, PINS_CHANNELS), "EN pins must be output-capable (<34)");
static_assert(_all_lt_34(DriverPins::CS, PINS_CHANNELS), "CS pins must be output-capable (<34)");
static_assert(MultiplexerPins::S0 < 34 && MultiplexerPins::S1 < 34 && MultiplexerPins::DIR < 34, "Multiplexer S0/S1/DIR must be output-capable (<34)");
// Encoder pins (34..39) are input-only by design → OK.

#endif  // PINS_H

/*
Usage (API-compatible):
  #include "Pins.h"
  pinMode(DriverPins::DIR[0], OUTPUT);
  pinMode(EncoderPins::SIGNAL[1], INPUT);
  digitalWrite(DriverPins::EN[2], HIGH);

Optional helpers (readability):
  using namespace pins_helpers;
  pinMode(dir(Chan::M0), OUTPUT);
  pinMode(step(1), OUTPUT);
  auto sck  = SPIPins::SCK;
  auto s0   = mux_s0();
  auto s1   = mux_s1();
  auto md   = mux_dir();
  auto p3v3 = power_3v3();
*/

#if false
static void handleMove_2(cmd* c)
{
    Command cmd(c);  // Create wrapper object

    // n (optional uses current), p (µm for linear, deg for rotary)
    int n = gCurrentMotor;
    if (cmd.getArg("n").isSet())
    {
        const int nv = cmd.getArg("n").getValue().toInt();
        if (nv >= 1 && nv <= kMotorCount)
        {
            n             = nv - 1;
            gCurrentMotor = static_cast<uint8_t>(n);
            applyUnitDefaults(gCurrentMotor);
        }
    }
    if (!gPC[n] || !gPC[n]->isEnabled())
    {
        Serial.println("[ERR] Motor not enabled");
        return;
    }

    //-----------------------------------------------------------------

    //-----------------------------------------------------------------
    const double p           = cmd.getArg("p").getValue().toDouble();
    int32_t      targetSteps = 0;

    if (isLinear(n))
    {
        // p in micrometers
        targetSteps = UnitConverter::convertFromMicrometers(p).TO_STEPS;
    }
    else
    {
        // p in degrees
        targetSteps = UnitConverter::convertFromDegrees(p).TO_STEPS;
    }

    //-----------------------------------------------------------------
    // Before any move, go to stable position (if not already there)
    int32_t stable = loadStableSteps(n);
    Serial.printf("[INFO] Last Stable Position:");
    Serial.printf("--- %ld steps\r\n", static_cast<long>(stable));

    if (isRotary(n))
    {
        // Convert steps → degrees, clamp to [0.05°, 359.95°], then back to steps
        double deg = UnitConverter::convertFromSteps(stable).TO_DEGREES;
        Serial.printf("--- %.2f°\r\n", deg);

        // --- Clamp stable range for rotary motors ---
        if (deg < 0.05)
            deg = 0.05;
        if (deg > 359.95)
            deg = 359.95;

        // wrap-around behavior (e.g., 370° → 10° instead of 359.95°):
        deg = fmod(deg, 360.0);
        if (deg < 0.0)
            deg += 360.0;

        stable = UnitConverter::convertFromDegrees(deg).TO_STEPS;
        Serial.printf("--- %.2f° (wrap-around)\r\n", deg);
    }

    //-----------------------------------------------------------------
    // Seed current from encoder as reference in the *current* revolution
    int32_t seedSteps;
    if (!seedCurrentFromEncoder(n, seedSteps))
    {
        Serial.printf("[ERR] ENC%d not responding ❌\r\n", n + 1);
    }

    const int32_t curSteps = gPC[n]->getCurrentSteps();

    // In handleMove(cmd) after computing `targetSteps`:
    const int32_t nowSteps = gPC[n]->getCurrentSteps();

    // Rotary two-phase logic
    if (isRotary(n))
    {
        const auto curCV = UnitConverter::convertFromSteps(nowSteps);
        const auto tgtCV = UnitConverter::convertFromSteps(targetSteps);
        double     ddeg  = fabs(tgtCV.TO_DEGREES - curCV.TO_DEGREES);

        if (ddeg > kTouchWindowDeg)
        {
            // Phase 1: Rapid to pre-target (target - sign*window)
            const double  dir      = (tgtCV.TO_DEGREES >= curCV.TO_DEGREES) ? +1.0 : -1.0;
            const double  preD     = tgtCV.TO_DEGREES - dir * kTouchWindowDeg;
            const int32_t preSteps = UnitConverter::convertFromDegrees(preD).TO_STEPS;

            // Save start point in case of power drop
            saveStableSteps(n, nowSteps);

            // Rapid profile
            setCustomProfile(*gPC[n], kFastMaxSpeed, kFastAccel);
            gPC[n]->moveToSteps(preSteps, MovementType::LONG_RANGE);

            // Chain Phase 2 (Touch) after homing-to-pre target:
            // Use a static callback (no lambdas) because attachOnComplete takes void(*)()
            static bool    s_pendingTouch = false;
            static uint8_t s_motorIdx     = 0;
            static int32_t s_finalTarget  = 0;

            s_pendingTouch = true;
            s_motorIdx     = n;
            s_finalTarget  = targetSteps;

            gPC[n]->attachOnComplete(
                []()
                {
                    // NOTE: if your attachOnComplete only accepts function pointers, replace this
                    // lambda with a static function and use globals: see previous message for pattern.
                });

            // If your PositionController requires a raw function pointer:
            extern void onRapidArrived();  // forward-declare a static function
            gPC[n]->attachOnComplete(onRapidArrived);

            Serial.printf("[MOVE] Rapid to pre-target (deg window=%.2f)\r\n", kTouchWindowDeg);
            return;
        }
        // Else small move → do single-phase touch:
        setCustomProfile(*gPC[n], kTouchMaxSpeed, kTouchAccel);
    }

    // Linear: normal distance-based config
    configureByDistance(*gPC[n], nowSteps, targetSteps);
    saveStableSteps(n, nowSteps);
    gPC[n]->moveToSteps(targetSteps, MovementType::MEDIUM_RANGE);

    if (curSteps != stable)
    {
        configureByDistance(*gPC[n], curSteps, stable);

        // 1. Move to stable
        gPC[n]->moveToSteps(stable, MovementType::MEDIUM_RANGE);

        // 2. When the motor reaches 'stable', automatically move to the requested target
        // Prepare next target for after homing
        gPendingTargetAfterHoming = true;
        gPendingTargetSteps       = targetSteps;
        // gPC[n]->attachOnComplete(onHomingComplete);

        // NOTE: We return after queuing homing-to-stable; user can issue move again or wait.
        Serial.printf("\r\n[INFO] Homing to stable first: stable (%ld steps), current (%ld steps)\r\n\r\n", static_cast<long>(stable), static_cast<long>(curSteps));
        return;
    }

    //-----------------------------------------------------------------
    // Seed current from encoder as reference in the *current* revolution
    int32_t seedSteps;
    if (!seedCurrentFromEncoder(n, seedSteps))
    {
        Serial.printf("[ERR] ENC%d not responding ❌\r\n", n + 1);
    }

    const int32_t nowSteps2 = gPC[n]->getCurrentSteps();
    configureByDistance(*gPC[n], nowSteps, targetSteps);

    // Persist "start position" immediately — used if power drops mid-move
    saveStableSteps(n, nowSteps2);

    // Open-loop move (per your logic). Hybrid path is available via CLI control command.
    if (gPC[n]->moveToSteps(targetSteps, MovementType::MEDIUM_RANGE))
    {
        Serial.printf("\r\n[MOVE] n=%d -> targetSteps=%ld (cur=%ld)\r\n", n + 1, static_cast<long>(targetSteps), static_cast<long>(nowSteps2));
    }
    else
    {
        Serial.println("[ERR] move command rejected ❌");
    }
}
#endif

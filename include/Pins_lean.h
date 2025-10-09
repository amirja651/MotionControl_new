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
static_assert(MultiplexerPins::S0 < 34 && MultiplexerPins::S1 < 34 && MultiplexerPins::DIR < 34,
              "Multiplexer S0/S1/DIR must be output-capable (<34)");
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
static_assert(MultiplexerPins::S0 < 34 && MultiplexerPins::S1 < 34 && MultiplexerPins::DIR < 34,
              "Multiplexer S0/S1/DIR must be output-capable (<34)");
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

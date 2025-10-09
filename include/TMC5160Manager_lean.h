// =============================
// File: TMC5160Manager.h
// Lean, allocation-free, RT-friendly rewrite
// =============================
#ifndef TMC5160_MANAGER_H
#define TMC5160_MANAGER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <cstdint>

// ---- Compile-time logging switch (0 = off, 1 = on)
#ifndef TMC_LOG
    #define TMC_LOG 0
#endif

#if TMC_LOG
    #define TMC_LOGI(fmt, ...)                                                                                         \
        do                                                                                                             \
        {                                                                                                              \
            Serial.printf((fmt), ##__VA_ARGS__);                                                                       \
        } while (0)
    #define TMC_LOGLN(s)                                                                                               \
        do                                                                                                             \
        {                                                                                                              \
            Serial.println((s));                                                                                       \
        } while (0)
#else
    #define TMC_LOGI(...)  ((void)0)
    #define TMC_LOGLN(...) ((void)0)
#endif

static constexpr float R_SENSE = 0.11f;  // Sense resistor value in ohms

struct MotorConfig
{
    uint16_t rms_current_mA{350};
    uint16_t irun{16};
    uint16_t ihold{8};
    uint16_t iholddelay{8};
    uint16_t microsteps{32};
};

// Derived class to access protected read/write
class TMC5160StepperExtended : public TMC5160Stepper
{
public:
    explicit TMC5160StepperExtended(uint16_t pinCS, float RS = R_SENSE) : TMC5160Stepper(pinCS, RS) {}
    using TMC5160Stepper::read;
    using TMC5160Stepper::write;
};

class TMC5160Manager
{
public:
    // Constructor (no dynamic allocation)
    TMC5160Manager(uint8_t driverIndex, uint16_t pinCS, float RS = R_SENSE) noexcept;

    bool begin() noexcept;  // init SPI driver & basic config
    bool testConnection(bool print = false) noexcept;

    struct DriverStatus
    {
        bool     connected{false};
        uint32_t version{0};
        uint32_t status{0};
        uint32_t stallGuard{0};
        uint32_t current{0};
        uint32_t temperature{0};
    };

    [[nodiscard]] DriverStatus getDriverStatus() noexcept;

    [[nodiscard]] uint16_t getMicrosteps() const noexcept
    {
        return _cfg.microsteps;
    }
    void configureDriver_All_Motors(bool useStealth) noexcept;
    void logDriverStatus() noexcept;  // enabled only if TMC_LOG=1
    void DriverOff() noexcept;

    [[nodiscard]] MotorConfig getMotorConfig() const noexcept
    {
        return _cfg;
    }

    // Current control methods (apply immediately if begun)
    void                   setRmsCurrent(uint16_t current_mA) noexcept;
    void                   setIrun(uint8_t irun) noexcept;
    void                   setIhold(uint8_t ihold) noexcept;
    void                   setMicrosteps(uint16_t microsteps) noexcept;
    [[nodiscard]] uint16_t getRmsCurrent() const noexcept
    {
        return _cfg.rms_current_mA;
    }
    [[nodiscard]] uint8_t getIrun() const noexcept
    {
        return static_cast<uint8_t>(_cfg.irun);
    }
    [[nodiscard]] uint8_t getIhold() const noexcept
    {
        return static_cast<uint8_t>(_cfg.ihold);
    }
    [[nodiscard]] uint16_t getMicrostepsConst() const noexcept
    {
        return _cfg.microsteps;
    }
    void applyCurrentSettings() noexcept;

    void                   setSGTHRS(uint32_t threshold) noexcept;  // 0x40
    [[nodiscard]] uint32_t getSG_RESULT() noexcept;                 // 0x41

private:
    bool configureDriver() noexcept;

    // Members ordered to avoid padding and ensure fast init
    TMC5160StepperExtended _driver;  // allocation-free
    uint8_t                _driverIndex{0};
    uint16_t               _pinCS{0};
    float                  _RS{R_SENSE};
    MotorConfig            _cfg{};

    static constexpr uint16_t DEFAULT_CURRENT_NEMA11_1004H = 350;  // mA
};

#endif  // TMC5160_MANAGER_H

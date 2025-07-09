#ifndef TMC5160_MANAGER_H
#define TMC5160_MANAGER_H

#include "Pins.h"
#include <Arduino.h>
#include <TMCStepper.h>

static constexpr float R_SENSE = 0.11f;  // Sense resistor value in ohms (updated value)

// Derived class to access protected methods
class TMC5160StepperExtended : public TMC5160Stepper
{
public:
    TMC5160StepperExtended(uint16_t pinCS, float RS = R_SENSE) : TMC5160Stepper(pinCS, RS) {}
    using TMC5160Stepper::read;
    using TMC5160Stepper::write;
};

class TMC5160Manager
{
public:
    // Constructor
    TMC5160Manager(uint8_t driverIndex, uint16_t pinCS, float RS = R_SENSE);

    // Initialization
    bool begin();
    bool testConnection(bool print = false);

    // Driver Status
    struct DriverStatus
    {
        bool     connected;
        uint32_t version;
        uint32_t status;
        uint32_t stallGuard;
        uint32_t current;
        uint32_t temperature;
    };

    DriverStatus getDriverStatus();

    // Motor Control
    void setCurrent(uint16_t current);
    void setMicrosteps(uint16_t microsteps);

    // Driver Configuration
    void configureDriver_All_Motors(bool useStealth);
    void logDriverStatus();
    void DriverOff();

private:
    TMC5160StepperExtended* _driver;

    uint8_t  _driverIndex;
    uint16_t _pinCS;
    float    _RS;

    uint16_t _rms_current_mA;
    uint16_t _irun;
    uint16_t _ihold;
    uint16_t _iholddelay;
    uint16_t _microsteps;

    static constexpr uint16_t DEFAULT_CURRENT_NEMA11_1004H = 700;  // Default current in mA
    static constexpr uint16_t MICROSTEPS_NEMA11_1004H      = 16;
    static constexpr uint16_t DEFAULT_CURRENT_PANCAKE      = 500;  // Default current in mA
    static constexpr uint16_t MICROSTEPS_PANCAKE           = 256;

    // Private helper methods
    bool     configureDriver();
    void     setSGTHRS(uint32_t threshold);
    uint32_t getSG_RESULT();
    uint8_t  calculateCurrentSetting(uint16_t desiredCurrent_mA, uint16_t rmsCurrent_mA);
};

#endif  // TMC5160_MANAGER_H
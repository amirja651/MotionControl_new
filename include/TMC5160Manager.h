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
    bool testConnection();

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

    // Motion Control
    void stopMotor();
    void emergencyStop();

    // Driver Configuration
    void configureDriver_Nema11_1004H(bool useStealth);
    void configureDriver_Pancake();
    void logDriverStatus();

private:
    TMC5160StepperExtended* _driver;

    uint8_t  _driverIndex;
    uint16_t _pinCS;
    float    _RS;

    static constexpr uint16_t DEFAULT_CURRENT = 700;  // Default current in mA
    static constexpr uint16_t MICROSTEPS      = 16;
    // Private helper methods
    bool     configureDriver();
    void     setSGTHRS(uint32_t threshold);
    uint32_t getSG_RESULT();
    void     DriverOff();
};

#endif  // TMC5160_MANAGER_H
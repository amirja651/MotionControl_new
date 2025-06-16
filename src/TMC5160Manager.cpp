#include "TMC5160Manager.h"
#include "Helper.h"

TMC5160Manager::TMC5160Manager(uint8_t driverIndex, uint16_t pinCS, float RS) : _driverIndex(driverIndex), _pinCS(pinCS), _RS(RS)
{
    _driver = nullptr;
}

bool TMC5160Manager::begin()
{
    // Create new driver instance
    _driver = new TMC5160StepperExtended(_pinCS, _RS);

    // Configure driver
    bool success = configureDriver();

    return success;
}

bool TMC5160Manager::testConnection()
{
    if (!_driver)
        return false;

    // Try to read the version register
    uint32_t version = _driver->version();
    // Turn off the driver
    DriverOff();
    return (version != 0 && version != 0xFFFFFFFF);
}

TMC5160Manager::DriverStatus TMC5160Manager::getDriverStatus()
{
    DriverStatus status = {false, 0, 0, 0, 0, 0};

    if (!_driver)
        return status;

    status.connected  = true;
    status.version    = _driver->version();
    status.status     = _driver->DRV_STATUS();
    status.stallGuard = _driver->TCOOLTHRS();

    // Safely get current value
    try
    {
        status.current = _driver->rms_current();
    }
    catch (...)
    {
        status.current = 0;
    }

    status.temperature = _driver->TSTEP();

    // Turn off the driver
    DriverOff();

    return status;
}

bool TMC5160Manager::configureDriver()
{
    if (!_driver)
        return false;

    // Basic configuration
    _driver->begin();
    delay(5);
    _driver->toff(5);  // Enable driver
    delay(5);
    _driver->rms_current(DEFAULT_CURRENT);  // Set current to 1A
    delay(5);
    _driver->microsteps(MICROSTEPS);  // Set microsteps to 16
    delay(5);
    // Configure spreadCycle
    _driver->en_pwm_mode(false);  // Disable stealthChop
    delay(5);
    _driver->pwm_autoscale(true);  // Enable automatic current scaling
    delay(5);
    // StallGuard configuration
    _driver->TCOOLTHRS(0xFFFFF);  // 20bit max
    delay(5);
    setSGTHRS(100);  // Stall threshold
    delay(5);

    if (!testConnection())
        return false;

    String gTrue  = Helper::greenText("true");
    String gFalse = Helper::redText("false");
    bool   sdMode = _driver->sd_mode();
    delay(5);
    bool drvEnn = _driver->drv_enn();
    delay(5);
    // Read and print important registers
    uint32_t drv_status = _driver->DRV_STATUS();
    delay(5);
    uint32_t gconf = _driver->read(0x00);
    delay(5);

    // Print configuration
    Serial.printf("[configureDriver] Driver %d configured:\n", _driverIndex);
    Serial.printf(" - Current: %d mA\n", DEFAULT_CURRENT);
    Serial.printf(" - Microsteps: %d\n", MICROSTEPS);
    Serial.printf(" - Mode: STEP/DIR with spreadCycle\n");
    Serial.printf(" - Hardware configured for Step & Dir mode: %s\n", sdMode ? gTrue.c_str() : gFalse.c_str());
    Serial.printf(" - Hardware enabled: %s\n", drvEnn ? gTrue.c_str() : gFalse.c_str());
    Serial.printf(" - DRV_STATUS: 0x%08X\n", drv_status);
    Serial.printf(" - GCONF: 0x%08X\n\n", gconf);

    // Turn off the driver
    DriverOff();

    return true;
}

void TMC5160Manager::configureDriver_Nema11_1004H()
{
    if (!_driver)
        return;

    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    // gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // StealthChop enable (initially)
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    _driver->GCONF(gconf);

    // ---------------------------
    // 2. Current Settings (Low Power Mode)
    // ---------------------------
    _driver->rms_current(DEFAULT_CURRENT);  // 0.7A RMS (safer for motor life)
    _driver->irun(16);
    _driver->ihold(8);
    _driver->iholddelay(8);   // 8 * 16 = 128 ms before going to hold current
    _driver->TPOWERDOWN(10);  // Power down delay (10 × 100ms) (irun -> ihold)

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    _driver->microsteps(16);  // Increased microstepping for smoother holding
    _driver->intpol(true);    // Smooth motion

    // ---------------------------
    // 4. StealthChop Settings (Enable for holding/low speed)
    // ---------------------------
    _driver->TPWMTHRS(0xFFFF);  // StealthChop active at low speeds (including holding)
    _driver->pwm_autoscale(true);
    _driver->pwm_autograd(true);
    _driver->pwm_ofs(36);
    _driver->pwm_grad(10);
    _driver->pwm_freq(2);
    _driver->en_pwm_mode(true);  // Enable StealthChop (silent mode) for holding

    // ---------------------------
    // 5. SpreadCycle Chopper Settings (used only at higher speeds)
    // ---------------------------
    // driver->toff(4);
    // driver->blank_time(24);
    // driver->hysteresis_start(3);
    // driver->hysteresis_end(1);

    // ---------------------------
    // 6. StallGuard & CoolStep
    // ---------------------------
    // driver->TCOOLTHRS(200);  // CoolStep threshold
    // driver->sgt(5);          // StallGuard threshold
    // driver->sfilt(true);

    // ---------------------------
    // 7. Motion Configuration (Soft Motion)
    // ---------------------------
    _driver->RAMPMODE(0);  // Positioning mode
    _driver->VSTART(1);    // Start slowly
    _driver->VSTOP(1);     // Stop slowly
    _driver->VMAX(600);    // Maximum speed, suitable for initial testing
    _driver->AMAX(100);    // Acceleration
    _driver->DMAX(100);    // Deceleration
    _driver->a1(300);      // Initial acceleration
    _driver->d1(300);      // Initial deceleration

    // Turn off the driver
    DriverOff();
}

void TMC5160Manager::configureDriver_Pancake()
{
    if (!_driver)
        return;

    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Internal Rsense
    gconf |= (1 << 2);  // StealthChop enable (initially)
    gconf |= (1 << 3);  // Microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Multistep filtering
    _driver->GCONF(gconf);

    // ---------------------------
    // 2. Current Settings (Low Power Mode)
    // ---------------------------
    _driver->rms_current(350);  // About 0.35A RMS (safe for Pancake)
    _driver->irun(200);         // Run current: ~0.35A
    _driver->ihold(100);        // Hold current: ~0.15A (increased for stability)
    _driver->iholddelay(1);     // Short delay before switching to ihold
    _driver->TPOWERDOWN(10);    // Power down delay

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    _driver->microsteps(16);  // Increased microstepping for smoother holding
    _driver->intpol(true);    // Smooth motion

    // ---------------------------
    // 4. StealthChop Settings (Enable for holding/low speed)
    // ---------------------------
    _driver->TPWMTHRS(0xFFFF);  // StealthChop active at low speeds (including holding)
    _driver->pwm_autoscale(true);
    _driver->pwm_autograd(true);
    _driver->pwm_ofs(36);
    _driver->pwm_grad(10);
    _driver->pwm_freq(2);
    _driver->en_pwm_mode(true);  // Enable StealthChop (silent mode) for holding

    // ---------------------------
    // 5. SpreadCycle Chopper Settings (used only at higher speeds)
    // ---------------------------
    _driver->toff(4);
    _driver->blank_time(24);
    _driver->hysteresis_start(3);
    _driver->hysteresis_end(1);

    // ---------------------------
    // 6. StallGuard & CoolStep
    // ---------------------------
    _driver->TCOOLTHRS(200);  // CoolStep threshold
    _driver->sgt(5);          // StallGuard threshold
    _driver->sfilt(true);

    // ---------------------------
    // 7. Motion Configuration (Soft Motion)
    // ---------------------------
    _driver->RAMPMODE(0);  // Positioning mode
    _driver->VSTART(1);    // Very soft start
    _driver->VSTOP(1);     // Smooth stop
    _driver->VMAX(600);    // Max speed (limit for Pancake)
    _driver->AMAX(100);    // Acceleration limit
    _driver->DMAX(100);    // Deceleration limit
    _driver->a1(300);      // Start acceleration
    _driver->d1(300);      // Start deceleration

    // Turn off the driver
    DriverOff();
}

void TMC5160Manager::DriverOff()
{
    digitalWrite(_pinCS, HIGH);
}

void TMC5160Manager::setCurrent(uint16_t current)
{
    if (!_driver)
        return;
    _driver->rms_current(current);

    // Turn off the driver
    DriverOff();
}

void TMC5160Manager::setMicrosteps(uint16_t microsteps)
{
    if (!_driver)
        return;
    _driver->microsteps(microsteps);

    // Turn off the driver
    DriverOff();
}

void TMC5160Manager::stopMotor() {}

void TMC5160Manager::emergencyStop()
{
    stopMotor();
    DriverOff();
}

void TMC5160Manager::setSGTHRS(uint32_t threshold)
{
    if (!_driver)
        return;
    _driver->write(0x40, threshold);  // Direct register access

    // Turn off the driver
    DriverOff();
}

uint32_t TMC5160Manager::getSG_RESULT()
{
    if (!_driver)
        return 0;
    return _driver->sg_result();  // TMC5160 uses lowercase method names

    // Turn off the driver
    DriverOff();
}

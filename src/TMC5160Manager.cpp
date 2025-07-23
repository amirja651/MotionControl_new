#include "TMC5160Manager.h"

TMC5160Manager::TMC5160Manager(uint8_t driverIndex, uint16_t pinCS, float RS) : _driverIndex(driverIndex), _pinCS(pinCS), _RS(RS)
{
    _driver = nullptr;
    if (driverIndex == 0)
    {
        _rms_current_mA = DEFAULT_CURRENT_NEMA11_1004H;
        _microsteps     = MICROSTEPS_NEMA11_1004H;
        _irun           = 16;  // calculateCurrentSetting(350, DEFAULT_CURRENT_NEMA11_1004H);
        _ihold          = 8;   // calculateCurrentSetting(180, DEFAULT_CURRENT_NEMA11_1004H);
        _iholddelay     = 8;
    }
    else
    {
        _rms_current_mA = 500;
        _microsteps     = DEFAULT_CURRENT_PANCAKE;
        _irun           = 12;  // 10
        _ihold          = 4;
        _iholddelay     = 1;
    }
}

bool TMC5160Manager::begin()
{
    // Create new driver instance
    _driver = new TMC5160StepperExtended(_pinCS, _RS);

    // Configure driver
    bool success = configureDriver();

    // Turn off the driver
    DriverOff();

    return success;
}

bool TMC5160Manager::testConnection(bool print)
{
    uint8_t  version  = _driver->version();
    uint32_t gconf    = _driver->GCONF();
    uint32_t status   = _driver->DRV_STATUS();
    uint32_t chopconf = _driver->CHOPCONF();
    _driver->GCONF(gconf);  // Write back the same value
    uint32_t readback = _driver->GCONF();

    delay(1);

    if (print)
    {
        Serial.println();
        Serial.print(F("[TestConnection] Driver: "));
        Serial.println(_driverIndex + 1);
    }

    static bool connection_failed = false;
    if (version == 0xFF || version == 0)
    {
        if (print)
        {
            Serial.print(F("Invalid version (0x"));
            Serial.print(version, HEX);
            Serial.println(F(")"));
        }
        connection_failed = true;
    }
    else if (gconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.println(F("GCONF register read failed"));
        }
        connection_failed = true;
    }
    else if (status == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.println(F("DRV_STATUS register read failed"));
        }
        connection_failed = true;
    }
    else if (chopconf == 0xFFFFFFFF)
    {
        if (print)
        {
            Serial.println(F("CHOPCONF register read failed"));
        }
        connection_failed = true;
    }
    else if (readback != gconf)
    {
        if (print)
        {
            Serial.println(F("GCONF register write/read mismatch"));
        }
    }
    if (connection_failed)
    {
        if (print)
        {
            Serial.println(F("connection failed!\r\n"));
            Serial.println();
        }
        DriverOff();
        return false;
    }

    if (print)
    {
        Serial.println(F("✅ connected successfully"));
        Serial.println();
    }

    DriverOff();
    return true;
}

TMC5160Manager::DriverStatus TMC5160Manager::getDriverStatus()
{
    DriverStatus status = {false, 0, 0, 0, 0, 0};

    status.connected   = true;
    status.version     = _driver->version();
    status.status      = _driver->DRV_STATUS();
    status.stallGuard  = _driver->TCOOLTHRS();
    status.current     = _driver->rms_current();
    status.temperature = _driver->TSTEP();

    DriverOff();
    return status;
}

bool TMC5160Manager::configureDriver()
{
    // Basic configuration
    _driver->begin();
    delay(5);

    _driver->toff(5);  // Enable driver (0 = StealthChop) mode (5 = SpreadCycle)
    delay(5);

    _driver->en_pwm_mode(true);    // Enable StealthChop
    _driver->pwm_autoscale(true);  // Enable automatic current scaling (StealthChop)
    _driver->pwm_autograd(true);
    _driver->TPWMTHRS(0xFFFF);  // Prevent jumping to SpreadCycle
    delay(5);

    _driver->rms_current(_rms_current_mA);  // Set current to 1A
    _driver->microsteps(_microsteps);       // Set microsteps to 16
    _driver->intpol(true);                  // Enable microstep interpolation
    delay(5);

    // StallGuard configuration
    _driver->TCOOLTHRS(0xFFFFF);  // 20bit max (0xFFFFF = 1023)
    setSGTHRS(100);               // Stall threshold (100 = 100% of current)

    if (!testConnection())
    {
        // Turn off the driver
        DriverOff();

        return false;
    }

    bool sd_mode = _driver->sd_mode();  // Check if driver is in STEP/DIR mode
    delay(5);
    bool drv_enn = _driver->drv_enn();  // Determines if the driver is enabled or not (from hardware perspective).
    delay(5);

    // Read and print important registers
    uint32_t drv_status = _driver->DRV_STATUS();
    delay(5);
    uint32_t gconf = _driver->read(0x00);
    delay(5);
    // Print configuration
    Serial.println();
    Serial.print(F("===========                                  [configureDriver] Driver                                    ===========\n"));
    Serial.printf("┌───────────┬──────────────┬────────────┬─────────────┬─────────────────┬──────────────────┬────────────┬────────────┐\n");
    Serial.printf("│ Driver Id │ Current (mA) │ Microsteps │     Mode    │ Step & Dir mode │ Hardware enabled │ DRV_STATUS │   GCONF    │\n");
    Serial.printf("├───────────┼──────────────┼────────────┼─────────────┼─────────────────┼──────────────────┼────────────┼────────────┤\n");
    Serial.printf("│ %-9d │ %-12d │ %-10d │ %-11s │ %-15d │ %-16d │ 0x%08X │ 0x%08X │\n", _driverIndex + 1, _rms_current_mA, _microsteps, "SpreadCycle", sd_mode ? 1 : 0, drv_enn ? 1 : 0, drv_status, gconf);
    Serial.printf("└───────────┴──────────────┴────────────┴─────────────┴─────────────────┴──────────────────┴────────────┴────────────┘\n");

    // Turn off the driver
    DriverOff();

    return true;
}

void TMC5160Manager::configureDriver_All_Motors(bool useStealth)
{
    // ---------------------------
    // 1. Basic Driver Configuration (GCONF)
    // ---------------------------
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Internal Rsense
    if (useStealth)
        gconf |= (1 << 2);  // StealthChop enable (initially)
    gconf |= (1 << 3);      // Microstep interpolation
    gconf |= (1 << 4);      // Double edge step
    gconf |= (1 << 6);      // Multistep filtering
    _driver->GCONF(gconf);
    delay(5);

    // ---------------------------
    // 2. Current Settings
    // ---------------------------
    _driver->rms_current(_rms_current_mA);
    _driver->irun(_irun);
    _driver->ihold(_ihold);
    _driver->iholddelay(_iholddelay);  // Short delay before switching to ihold
    _driver->TPOWERDOWN(10);           // Power down delay
    delay(5);

    // ---------------------------
    // 3. Microstepping & Interpolation
    // ---------------------------
    _driver->microsteps(_microsteps);  // Set microsteps to 16
    _driver->intpol(true);             // Enable microstep interpolation
    delay(5);

    // ---------------------------
    // 4. StealthChop Settings (Enable for holding/low speed)
    // ---------------------------
    _driver->en_pwm_mode(useStealth);  // Enable StealthChop
    _driver->pwm_autoscale(true);      // Enable automatic current scaling (StealthChop)
    _driver->pwm_autograd(true);       // Enable automatic gradient (StealthChop)
    _driver->TPWMTHRS(0xFFFF);         // StealthChop active at all speeds
    if (useStealth)
    {
        _driver->pwm_ofs(36);   // Offset
        _driver->pwm_grad(10);  // Gradient
        _driver->pwm_freq(2);   // Frequency
    }
    delay(5);

    // ---------------------------
    // 5. SpreadCycle Chopper Settings (used only at higher speeds)
    // ---------------------------
    if (!useStealth)
    {
        _driver->toff(4);              // Enable SpreadCycle
        _driver->blank_time(24);       // Blank time
        _driver->hysteresis_start(3);  // Hysteresis start
        _driver->hysteresis_end(1);    // Hysteresis end
    }
    delay(5);

    if (_driverIndex != 0)  // only for pancake
    {
        // ---------------------------
        // 6. StallGuard & CoolStep
        // ---------------------------
        _driver->TCOOLTHRS(200);  // CoolStep threshold
        _driver->sgt(5);          // StallGuard threshold
        _driver->sfilt(true);
        delay(5);
    }
}

void TMC5160Manager::DriverOff()
{
    // Use digitalWrite instead of gpio_set_level for safety
    digitalWrite(_pinCS, HIGH);
    delay(5);
}

void TMC5160Manager::setSGTHRS(uint32_t threshold)
{
    _driver->write(0x40, threshold);  // Direct register access
    delay(5);

    // Turn off the driver
    DriverOff();
}

void TMC5160Manager::logDriverStatus()
{
    uint32_t version       = _driver->version();
    bool     driverEnabled = !_driver->drv_enn();  // ENN pin: active LOW
    bool     stealthChop   = _driver->en_pwm_mode();
    uint32_t drv_status    = _driver->DRV_STATUS();
    uint32_t gconf         = _driver->GCONF();
    uint16_t current       = _driver->rms_current();
    uint8_t  irun          = _driver->irun();
    uint8_t  ihold         = _driver->ihold();
    uint16_t tpwmthrs      = _driver->TPWMTHRS();
    // Decode DRV_STATUS bits
    bool otpw = drv_status & (1UL << 26);
    bool ot   = drv_status & (1UL << 25);
    bool s2ga = drv_status & (1UL << 24);
    bool s2gb = drv_status & (1UL << 22);
    bool ola  = drv_status & (1UL << 18);
    bool olb  = drv_status & (1UL << 16);

    // Header row
    Serial.println();
    Serial.printf("===========                       [LogDriverStatus]                      ===========\n");
    Serial.printf("┌─────────────────────┬────────────────────────────────────────────────────────────┐\n");
    Serial.printf("│ Parameter           │ Value                                                      │\n");
    Serial.printf("├─────────────────────┼────────────────────────────────────────────────────────────┤\n");
    Serial.printf("│ Driver Id           │ %d                                                         │\n", _driverIndex + 1);
    Serial.printf("│ Version             │ 0x%08X                                                     │\n", version);
    Serial.printf("│ Driver Enabled      │ %s                                                         │\n", driverEnabled ? "YES" : "NO");
    Serial.printf("│ StealthChop Mode    │ %s                                                         │\n", stealthChop ? "YES" : "NO");
    Serial.printf("│ DRV_STATUS          │ 0x%08X                                                     │\n", drv_status);
    Serial.printf("│ GCONF               │ 0x%08X                                                     │\n", gconf);
    Serial.printf("│ RMS Current         │ %d mA                                                      │\n", current);
    Serial.printf("│ IRUN/IHOLD          │ %d / %d                                                    │\n", irun, ihold);
    Serial.printf("│ TPWMTHRS            │ %u                                                         │\n", tpwmthrs);
    Serial.printf("│ Overtemp Warning    │ %s                                                         │\n", otpw ? "YES" : "NO");
    Serial.printf("│ Overtemp Shutdown   │ %s                                                         │\n", ot ? "YES" : "NO");
    Serial.printf("│ Short to GND A      │ %s                                                         │\n", s2ga ? "YES" : "NO");
    Serial.printf("│ Short to GND B      │ %s                                                         │\n", s2gb ? "YES" : "NO");
    Serial.printf("│ Open Load A         │ %s                                                         │\n", ola ? "YES" : "NO");
    Serial.printf("│ Open Load B         │ %s                                                         │\n", olb ? "YES" : "NO");
    Serial.printf("└─────────────────────┴────────────────────────────────────────────────────────────┘\n");
}

// Convert target current (mA) to 5-bit value for irun or ihold (between 0 and 31)
uint8_t TMC5160Manager::calculateCurrentSetting(uint16_t desiredCurrent_mA, uint16_t rmsCurrent_mA)
{
    float ratio = (float)desiredCurrent_mA / (float)rmsCurrent_mA;
    int   value = round(ratio * 31.0f);

    // Limit to valid range [0, 31]
    if (value < 0)
        value = 0;
    if (value > 31)
        value = 31;

    return (uint8_t)value;
}

uint16_t TMC5160Manager::getMicrosteps()
{
    return _microsteps;
}

MotorConfig TMC5160Manager::getMotorConfig()
{
    MotorConfig config;
    config.rms_current_mA = _rms_current_mA;
    config.irun           = _irun;
    config.ihold          = _ihold;
    config.iholddelay     = _iholddelay;
    config.microsteps     = _microsteps;
    return config;
}

// Current control methods
void TMC5160Manager::setRmsCurrent(uint16_t current_mA)
{
    _rms_current_mA = current_mA;
    if (_driver != nullptr)
    {
        _driver->rms_current(_rms_current_mA);
        delay(5);
    }
}

void TMC5160Manager::setIrun(uint8_t irun)
{
    _irun = irun;
    if (_driver != nullptr)
    {
        _driver->irun(_irun);
        delay(5);
    }
}

void TMC5160Manager::setIhold(uint8_t ihold)
{
    _ihold = ihold;
    if (_driver != nullptr)
    {
        _driver->ihold(_ihold);
        delay(5);
    }
}

void TMC5160Manager::setMicrosteps(uint16_t microsteps)
{
    _microsteps = microsteps;
    if (_driver != nullptr)
    {
        _driver->microsteps(_microsteps);
        delay(5);
    }
}

uint16_t TMC5160Manager::getRmsCurrent() const
{
    return _rms_current_mA;
}

uint8_t TMC5160Manager::getIrun() const
{
    return _irun;
}

uint8_t TMC5160Manager::getIhold() const
{
    return _ihold;
}

uint16_t TMC5160Manager::getMicrosteps() const
{
    return _microsteps;
}

void TMC5160Manager::applyCurrentSettings()
{
    if (_driver != nullptr)
    {
        _driver->rms_current(_rms_current_mA);
        _driver->irun(_irun);
        _driver->ihold(_ihold);
        delay(5);
    }
}
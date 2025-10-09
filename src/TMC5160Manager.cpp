#include "TMC5160Manager_lean.h"  // driver manager

// =============================
// File: TMC5160Manager.cpp
// =============================
// Small helper: micro delays without blocking long
static inline void tiny_delay_us(uint32_t us)
{
    if (us)
        delayMicroseconds(us);
}

TMC5160Manager::TMC5160Manager(uint8_t driverIndex, uint16_t pinCS, float RS) noexcept
    : _driver(pinCS, RS), _driverIndex(driverIndex), _pinCS(pinCS), _RS(RS)
{
    // Select sane defaults per driver index
    if (driverIndex == 0)
    {
        _cfg.rms_current_mA = DEFAULT_CURRENT_NEMA11_1004H;
        _cfg.microsteps     = 32;  // NEMA11: quieter & smooth
        _cfg.irun           = 16;
        _cfg.ihold          = 8;
        _cfg.iholddelay     = 8;
    }
    else
    {
        _cfg.rms_current_mA = 500;
        _cfg.microsteps     = 64;
        _cfg.irun           = 12;
        _cfg.ihold          = 4;
        _cfg.iholddelay     = 1;
    }
}

bool TMC5160Manager::begin() noexcept
{
    pinMode(_pinCS, OUTPUT);
    digitalWrite(_pinCS, HIGH);

    _driver.begin();
    tiny_delay_us(50);

    const bool ok = configureDriver();
    DriverOff();
    return ok;
}

bool TMC5160Manager::testConnection(bool print) noexcept
{
    const uint8_t  version  = _driver.version();
    const uint32_t gconf    = _driver.GCONF();
    const uint32_t status   = _driver.DRV_STATUS();
    const uint32_t chopconf = _driver.CHOPCONF();
    _driver.GCONF(gconf);  // write-back
    const uint32_t readback = _driver.GCONF();

    tiny_delay_us(50);

    bool ok = true;
    if (version == 0u || version == 0xFFu)
        ok = false;
    if (gconf == 0xFFFFFFFFu)
        ok = false;
    if (status == 0xFFFFFFFFu)
        ok = false;
    if (chopconf == 0xFFFFFFFFu)
        ok = false;
    if (readback != gconf)
        ok = false;

    if (print && TMC_LOG)
    {
        if (!ok)
        {
            TMC_LOGLN("[TMC5160] connection failed");
        }
        else
        {
            TMC_LOGLN("[TMC5160] connected OK");
        }
    }

    DriverOff();
    return ok;
}

TMC5160Manager::DriverStatus TMC5160Manager::getDriverStatus() noexcept
{
    DriverStatus s{};
    s.connected   = true;
    s.version     = _driver.version();
    s.status      = _driver.DRV_STATUS();
    s.stallGuard  = _driver.TCOOLTHRS();  // user-threshold used
    s.current     = _driver.rms_current();
    s.temperature = _driver.TSTEP();  // library proxy
    DriverOff();
    return s;
}

bool TMC5160Manager::configureDriver() noexcept
{
    // Enable SpreadCycle timing base (toff>0) then StealthChop tuning
    _driver.toff(5);  // 0 = off, >0 enables driver
    tiny_delay_us(50);

    _driver.en_pwm_mode(true);  // StealthChop
    _driver.pwm_autoscale(true);
    _driver.pwm_autograd(true);
    _driver.TPWMTHRS(0xFFFF);  // keep StealthChop active
    tiny_delay_us(50);

    _driver.rms_current(_cfg.rms_current_mA);
    _driver.microsteps(_cfg.microsteps);
    _driver.intpol(true);
    tiny_delay_us(50);

    _driver.TCOOLTHRS(0xFFFFF);  // 20-bit max
    setSGTHRS(100);

    const bool ok = testConnection(false);
    if (!ok)
    {
        DriverOff();
        return false;
    }

    DriverOff();
    return true;
}

void TMC5160Manager::configureDriver_All_Motors(bool useStealth) noexcept
{
    // 1) GCONF
    uint32_t gconf = 0;
    gconf |= (1u << 0);  // internal Rsense
    if (useStealth)
        gconf |= (1u << 2);  // StealthChop enable
    gconf |= (1u << 3);      // microstep interpolation
    gconf |= (1u << 4);      // double edge step
    gconf |= (1u << 6);      // multistep filtering
    _driver.GCONF(gconf);
    tiny_delay_us(50);

    // 2) Currents
    _driver.rms_current(_cfg.rms_current_mA);
    _driver.irun(static_cast<uint8_t>(_cfg.irun));
    _driver.ihold(static_cast<uint8_t>(_cfg.ihold));
    _driver.iholddelay(static_cast<uint8_t>(_cfg.iholddelay));
    _driver.TPOWERDOWN(10);
    tiny_delay_us(50);

    // 3) Microstepping
    _driver.microsteps(_cfg.microsteps);
    _driver.intpol(true);
    tiny_delay_us(50);

    // 4) StealthChop
    _driver.en_pwm_mode(useStealth);
    _driver.pwm_autoscale(true);
    _driver.pwm_autograd(true);
    _driver.TPWMTHRS(0xFFFF);
    if (useStealth)
    {
        _driver.pwm_ofs(36);
        _driver.pwm_grad(10);
        _driver.pwm_freq(2);
    }
    tiny_delay_us(50);

    // 5) SpreadCycle (only if not using StealthChop)
    if (!useStealth)
    {
        _driver.toff(4);
        _driver.blank_time(24);
        _driver.hysteresis_start(3);
        _driver.hysteresis_end(1);
    }
    tiny_delay_us(50);

    // 6) StallGuard/CoolStep (skip for index 0 if not needed)
    if (_driverIndex != 0)
    {
        _driver.TCOOLTHRS(200);
        _driver.sgt(5);
        _driver.sfilt(true);
        tiny_delay_us(50);
    }
}

void TMC5160Manager::DriverOff() noexcept
{
    // CS high is a safe idle state for the SPI device.
    digitalWrite(_pinCS, HIGH);
    tiny_delay_us(10);
}

void TMC5160Manager::setSGTHRS(uint32_t threshold) noexcept
{
    _driver.write(0x40, threshold & 0xFFFFu);
    tiny_delay_us(20);
    DriverOff();
}

uint32_t TMC5160Manager::getSG_RESULT() noexcept
{
    const uint32_t v = _driver.read(0x41);
    DriverOff();
    return v;
}

void TMC5160Manager::logDriverStatus() noexcept
{
#if TMC_LOG
    const uint32_t version  = _driver.version();
    const bool     enabled  = !_driver.drv_enn();  // ENN active LOW
    const bool     stealth  = _driver.en_pwm_mode();
    const uint32_t drv_stat = _driver.DRV_STATUS();
    const uint32_t gconf    = _driver.GCONF();
    const uint16_t current  = _driver.rms_current();
    const uint8_t  irun     = _driver.irun();
    const uint8_t  ihold    = _driver.ihold();
    const uint16_t tpwmthrs = _driver.TPWMTHRS();

    TMC_LOGLN("[TMC5160] ---- status ----");
    TMC_LOGI("id=%u ver=0x%08X enabled=%u stealth=%u drv=0x%08X gconf=0x%08X\n",
             static_cast<unsigned>(_driverIndex + 1),
             version,
             enabled,
             stealth,
             drv_stat,
             gconf);
    TMC_LOGI("rms=%u irun=%u ihold=%u tpwm=%u\n", current, irun, ihold, tpwmthrs);
#endif
}

void TMC5160Manager::setRmsCurrent(uint16_t current_mA) noexcept
{
    _cfg.rms_current_mA = current_mA;
    _driver.rms_current(_cfg.rms_current_mA);
    tiny_delay_us(20);
}

void TMC5160Manager::setIrun(uint8_t irun) noexcept
{
    _cfg.irun = irun;
    _driver.irun(static_cast<uint8_t>(_cfg.irun));
    tiny_delay_us(20);
}

void TMC5160Manager::setIhold(uint8_t ihold) noexcept
{
    _cfg.ihold = ihold;
    _driver.ihold(static_cast<uint8_t>(_cfg.ihold));
    tiny_delay_us(20);
}

void TMC5160Manager::setMicrosteps(uint16_t microsteps) noexcept
{
    _cfg.microsteps = microsteps;
    _driver.microsteps(_cfg.microsteps);
    tiny_delay_us(20);
}

void TMC5160Manager::applyCurrentSettings() noexcept
{
    _driver.rms_current(_cfg.rms_current_mA);
    _driver.irun(static_cast<uint8_t>(_cfg.irun));
    _driver.ihold(static_cast<uint8_t>(_cfg.ihold));
    tiny_delay_us(20);
}

#pragma once
/**
 * @file    TMC5160Module.h
 * @brief   Multi-motor TMC5160/TMC5160T manager with AccelStepper Step/Dir pulse generation (non-blocking).
 *
 * Compliance:
 * - Comments in English; MISRA C++ minded; RAII for mutex lock; no dynamic allocation in RT path.
 * - Non-blocking: only AccelStepper::run()/runSpeed() in tick(); no delays in hot path.
 * - SPI shared bus; each TMC5160 has its own CS.
 * - Open-loop motion; homing is logical via API (user handles sensors externally).
 */

#include <AccelStepper.h>
#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>  // Ensure library is installed
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cmath>
#include <new>  // for std::nothrow
#include <vector>

// ========================= Types & Constants =========================

enum class MotorKind : uint8_t { Rotational = 0U, Linear = 1U, Other = 2U };

enum class EnablePolarity : uint8_t { ActiveLow = 0U, ActiveHigh = 1U };

enum class RunMode : uint8_t {
  Position = 0U,  // AccelStepper::run()
  Constant = 1U   // AccelStepper::runSpeed()
};

struct MotorConfig final {
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t cs_pin;
  int8_t en_pin;  // -1 if not wired
  EnablePolarity en_polarity;
  bool invert_dir;
  MotorKind kind;
  const char* name;  // nullable

  uint16_t steps_per_rev;        // e.g., 200
  uint16_t microsteps;           // e.g., 16 or 32
  uint16_t micrometers_per_rev;  // linear: 1 rev = 200 µm by spec

  uint16_t hold_current_mA;  // logical hold current
  uint16_t run_current_mA;   // logical run current (RMS)
  float r_sense_ohm;         // e.g., 0.075f

  double min_user;  // soft limit min (deg or µm)
  double max_user;  // soft limit max (deg or µm)

  float max_speed_sps;   // steps per second
  float max_accel_sps2;  // steps per second^2
};

struct ProbeResult final {
  const char* name;
  bool connected;
  uint8_t cs_pin;
};

// ========================= RAII Mutex Guard =========================

class MutexGuard final {
 public:
  explicit MutexGuard(SemaphoreHandle_t m) : m_mtx(m) { (void)xSemaphoreTake(m_mtx, portMAX_DELAY); }
  ~MutexGuard() { (void)xSemaphoreGive(m_mtx); }
  MutexGuard(const MutexGuard&) = delete;
  MutexGuard& operator=(const MutexGuard&) = delete;

 private:
  SemaphoreHandle_t m_mtx;
};

// ========================= TMC5160Motor =============================

class TMC5160Motor final {
 public:
  TMC5160Motor(const MotorConfig& cfg, SPIClass& spi_bus)
      : m_cfg(cfg),
        m_spi(spi_bus),
        m_stepper(AccelStepper::DRIVER, cfg.step_pin, cfg.dir_pin),
        m_driver(nullptr),
        m_mutex(nullptr),
        m_connected(false),
        m_run_mode(RunMode::Position) {}

  ~TMC5160Motor() {
    if (m_mutex != nullptr) {
      vSemaphoreDelete(m_mutex);
      m_mutex = nullptr;
    }
    delete m_driver;  // allocated in begin()
    m_driver = nullptr;
  }

  TMC5160Motor(const TMC5160Motor&) = delete;
  TMC5160Motor& operator=(const TMC5160Motor&) = delete;

  /**
   * @brief Initialize pins, mutex, driver (SPI), and AccelStepper profile.
   * @return true on success.
   * Note: Any heap allocation happens only here (driver, mutex). No RT allocations.
   */
  bool begin() {
    m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == nullptr) {
      return false;
    }

    pinMode(m_cfg.step_pin, OUTPUT);
    pinMode(m_cfg.dir_pin, OUTPUT);
    pinMode(m_cfg.cs_pin, OUTPUT);
    digitalWrite(m_cfg.cs_pin, HIGH);  // CS idle

    if (m_cfg.en_pin >= 0) {
      pinMode(static_cast<uint8_t>(m_cfg.en_pin), OUTPUT);
      driverEnable(false);  // keep disabled until configured
    }

    m_stepper.setPinsInverted(m_cfg.invert_dir, false, false);
    m_stepper.setMaxSpeed(m_cfg.max_speed_sps);
    m_stepper.setAcceleration(m_cfg.max_accel_sps2);

    // Create driver on shared SPI — init-time allocation only
    m_driver = new (std::nothrow) TMC5160Stepper(m_cfg.cs_pin);
    if (m_driver == nullptr) {
      return false;
    }

    // --- Driver configuration ---
    m_driver->begin();  // SPI/session init
    m_driver->microsteps(m_cfg.microsteps);

    // Run current (RMS) and Hold ratio (safe, portable to 5160)
    m_driver->rms_current(m_cfg.run_current_mA);
    {
      const float den = (m_cfg.run_current_mA == 0U) ? 1.0f : static_cast<float>(m_cfg.run_current_mA);
      float ratio = static_cast<float>(m_cfg.hold_current_mA) / den;  // 0..1
      if (ratio < 0.0f) {
        ratio = 0.0f;
      }
      if (ratio > 1.0f) {
        ratio = 1.0f;
      }
      m_driver->hold_multiplier(ratio);
      m_driver->TPOWERDOWN(20U);  // optional idle powerdown delay
    }

    // Chopper & timing defaults (conservative, widely compatible)
    m_driver->toff(5U);
    m_driver->blank_time(24U);

    // Keep StealthChop disabled by default for deterministic step response:
    m_driver->TPWMTHRS(0U);  // no StealthChop threshold set

    // Probe connectivity (uses test_connection(): 0 => OK)
    m_connected = probeDriver_();

    return true;
  }

  void driverEnable(bool enable) noexcept {
    if (m_cfg.en_pin < 0) {
      return;
    }
    const bool active = (m_cfg.en_polarity == EnablePolarity::ActiveLow) ? (!enable) : (enable);
    digitalWrite(static_cast<uint8_t>(m_cfg.en_pin), active ? LOW : HIGH);
  }

  bool isConnected() const noexcept { return m_connected; }

  bool reprobe() {
    MutexGuard g(m_mutex);
    m_connected = probeDriver_();
    return m_connected;
  }

  void setRunMode(RunMode mode) noexcept {
    MutexGuard g(m_mutex);
    m_run_mode = mode;
  }

  inline void tick() noexcept {
    MutexGuard g(m_mutex);
    if (m_run_mode == RunMode::Position) {
      (void)m_stepper.run();
    } else {
      (void)m_stepper.runSpeed();
    }
  }

  // ---------------------- Motion Commands ----------------------

  bool moveToUser(double target_user) {
    MutexGuard g(m_mutex);
    if (!withinLimits_(target_user)) {
      return false;
    }
    const int32_t steps = userToSteps_(target_user);
    m_stepper.moveTo(steps);
    m_run_mode = RunMode::Position;
    return true;
  }

  void setSpeedUser(double speed_user_per_s) {
    MutexGuard g(m_mutex);
    const float sps = userSpeedToStepsPerSecond_(speed_user_per_s);
    m_stepper.setSpeed(sps);
    m_run_mode = RunMode::Constant;
  }

  void softStop() {
    MutexGuard g(m_mutex);
    m_stepper.stop();
  }

  void setCurrentPositionUser(double user_value) {
    MutexGuard g(m_mutex);
    const int32_t steps = userToSteps_(clampUser_(user_value));
    m_stepper.setCurrentPosition(steps);
  }

  void zeroAtCurrent() {
    MutexGuard g(m_mutex);
    m_stepper.setCurrentPosition(0);
  }

  double currentPositionUser() {
    MutexGuard g(m_mutex);
    return stepsToUser_(static_cast<int32_t>(m_stepper.targetPosition()));
  }

  double targetPositionUser() {
    MutexGuard g(m_mutex);
    return stepsToUser_(static_cast<int32_t>(m_stepper.targetPosition()));
  }

  // ---------------------- Configuration ----------------------

  void setSoftLimits(double min_user, double max_user) {
    MutexGuard g(m_mutex);
    m_cfg.min_user = min_user;
    m_cfg.max_user = max_user;
  }

  void setMaxSpeedStepsPerSec(float sps) {
    MutexGuard g(m_mutex);
    m_cfg.max_speed_sps = sps;
    m_stepper.setMaxSpeed(sps);
  }

  void setAccelerationStepsPerSec2(float sps2) {
    MutexGuard g(m_mutex);
    m_cfg.max_accel_sps2 = sps2;
    m_stepper.setAcceleration(sps2);
  }

  void setMicrosteps(uint16_t microsteps) {
    MutexGuard g(m_mutex);
    m_cfg.microsteps = microsteps;
    if (m_driver != nullptr) {
      m_driver->microsteps(microsteps);
    }
  }

  void setRunCurrent_mA(uint16_t mA) {
    MutexGuard g(m_mutex);
    m_cfg.run_current_mA = mA;
    if (m_driver != nullptr) {
      m_driver->rms_current(mA);
      // Recompute hold ratio to keep semantics consistent
      const float den = (m_cfg.run_current_mA == 0U) ? 1.0f : static_cast<float>(m_cfg.run_current_mA);
      float ratio = static_cast<float>(m_cfg.hold_current_mA) / den;
      if (ratio < 0.0f) {
        ratio = 0.0f;
      }
      if (ratio > 1.0f) {
        ratio = 1.0f;
      }
      m_driver->hold_multiplier(ratio);
    }
  }

  void setHoldCurrent_mA(uint16_t mA) {
    MutexGuard g(m_mutex);
    m_cfg.hold_current_mA = mA;
    if (m_driver != nullptr) {
      const float den = (m_cfg.run_current_mA == 0U) ? 1.0f : static_cast<float>(m_cfg.run_current_mA);
      float ratio = static_cast<float>(mA) / den;
      if (ratio < 0.0f) {
        ratio = 0.0f;
      }
      if (ratio > 1.0f) {
        ratio = 1.0f;
      }
      m_driver->hold_multiplier(ratio);
    }
  }

  const MotorConfig& config() const noexcept { return m_cfg; }

 private:
  // --- data ---
  MotorConfig m_cfg;
  SPIClass& m_spi;
  AccelStepper m_stepper;
  TMC5160Stepper* m_driver;  // created in begin()
  SemaphoreHandle_t m_mutex;
  bool m_connected;
  RunMode m_run_mode;

  // --- constants ---
  static constexpr double kDegPerRev = 360.0;
  static constexpr double kLinearDefaultUmPerRev = 200.0;  // per specification

  // --- helpers ---
  static inline double wrapDegrees_(double deg) noexcept {
    double d = std::fmod(deg, kDegPerRev);
    if (d < 0.0) {
      d += kDegPerRev;
    }
    return d;
  }

  inline bool withinLimits_(double user) const noexcept {
    if (m_cfg.kind == MotorKind::Rotational) {
      const double u = wrapDegrees_(user);
      return (u >= m_cfg.min_user) && (u <= m_cfg.max_user);
    }
    if (m_cfg.kind == MotorKind::Linear) {
      return (user >= m_cfg.min_user) && (user <= m_cfg.max_user);
    }
    (void)user;
    return true;  // Other: no limits by default
  }

  inline double clampUser_(double user) const noexcept {
    if (m_cfg.kind == MotorKind::Rotational) {
      const double w = wrapDegrees_(user);
      return (w < m_cfg.min_user) ? m_cfg.min_user : ((w > m_cfg.max_user) ? m_cfg.max_user : w);
    }
    if (m_cfg.kind == MotorKind::Linear) {
      return (user < m_cfg.min_user) ? m_cfg.min_user : ((user > m_cfg.max_user) ? m_cfg.max_user : user);
    }
    return user;
  }

  inline int32_t userToSteps_(double user) const noexcept {
    const int32_t steps_per_mech_rev =
        static_cast<int32_t>(m_cfg.steps_per_rev) * static_cast<int32_t>(m_cfg.microsteps);

    if (m_cfg.kind == MotorKind::Rotational) {
      const double deg = wrapDegrees_(user);
      const double revs = deg / kDegPerRev;
      const double steps = revs * static_cast<double>(steps_per_mech_rev);
      return static_cast<int32_t>(std::lround(steps));
    }

    if (m_cfg.kind == MotorKind::Linear) {
      const double um_per_rev =
          (m_cfg.micrometers_per_rev > 0U) ? static_cast<double>(m_cfg.micrometers_per_rev) : kLinearDefaultUmPerRev;
      const double revs = user / um_per_rev;
      const double steps = revs * static_cast<double>(steps_per_mech_rev);
      return static_cast<int32_t>(std::lround(steps));
    }

    return static_cast<int32_t>(std::lround(user));  // Other: raw steps
  }

  inline double stepsToUser_(int32_t steps) const noexcept {
    const int32_t steps_per_mech_rev =
        static_cast<int32_t>(m_cfg.steps_per_rev) * static_cast<int32_t>(m_cfg.microsteps);

    if (m_cfg.kind == MotorKind::Rotational) {
      const double revs = static_cast<double>(steps) / static_cast<double>(steps_per_mech_rev);
      double deg = revs * kDegPerRev;
      deg = wrapDegrees_(deg);
      return deg;
    }

    if (m_cfg.kind == MotorKind::Linear) {
      const double um_per_rev =
          (m_cfg.micrometers_per_rev > 0U) ? static_cast<double>(m_cfg.micrometers_per_rev) : kLinearDefaultUmPerRev;
      const double revs = static_cast<double>(steps) / static_cast<double>(steps_per_mech_rev);
      return revs * um_per_rev;
    }

    return static_cast<double>(steps);  // Other
  }

  inline float userSpeedToStepsPerSecond_(double user_per_sec) const noexcept {
    if (m_cfg.kind == MotorKind::Rotational) {
      const double revs_per_s = user_per_sec / kDegPerRev;
      const double sps = revs_per_s * static_cast<double>(m_cfg.steps_per_rev) * static_cast<double>(m_cfg.microsteps);
      return static_cast<float>(sps);
    }
    if (m_cfg.kind == MotorKind::Linear) {
      const double um_per_rev =
          (m_cfg.micrometers_per_rev > 0U) ? static_cast<double>(m_cfg.micrometers_per_rev) : kLinearDefaultUmPerRev;
      const double revs_per_s = user_per_sec / um_per_rev;
      const double sps = revs_per_s * static_cast<double>(m_cfg.steps_per_rev) * static_cast<double>(m_cfg.microsteps);
      return static_cast<float>(sps);
    }
    return static_cast<float>(user_per_sec);  // Other: already steps/s
  }

  bool probeDriver_() {
    if (m_driver == nullptr) {
      return false;
    }
    const uint8_t res = m_driver->test_connection();  // 0 => OK
    return (res == 0U);
  }
};

// ========================= TMC5160BusManager ========================

class TMC5160BusManager final {
 public:
  explicit TMC5160BusManager(SPIClass& spi_bus) : m_spi(spi_bus) {}

  void addMotor(TMC5160Motor* m) {
    if (m != nullptr) {
      m_motors.push_back(m);
    }  // init-time only
  }

  bool begin(uint8_t sck, uint8_t miso, uint8_t mosi, uint32_t spi_hz = 1000000UL) {
    m_spi.begin(sck, miso, mosi);
    m_spi.beginTransaction(SPISettings(spi_hz, MSBFIRST, SPI_MODE3));
    m_spi.endTransaction();

    bool ok = true;
    for (auto* m : m_motors) {
      if (!(m->begin())) {
        ok = false;
      }
    }
    return ok;
  }

  void serviceAll() {
    for (auto* m : m_motors) {
      m->tick();
    }
  }

  size_t probeAll(ProbeResult* out, size_t max_count) {
    size_t n = 0U;
    for (auto* m : m_motors) {
      if (n >= max_count) {
        break;
      }
      const bool conn = m->reprobe();
      out[n] = ProbeResult{m->config().name, conn, m->config().cs_pin};
      ++n;
    }
    return n;
  }

  TMC5160Motor* at(size_t idx) const {
    if (idx >= m_motors.size()) {
      return nullptr;
    }
    return m_motors[idx];
  }

  size_t size() const noexcept { return m_motors.size(); }

 private:
  SPIClass& m_spi;
  std::vector<TMC5160Motor*> m_motors;  // built at init; no RT allocations
};

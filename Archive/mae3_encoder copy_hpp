#pragma once
#include <Arduino.h>

#include <atomic>
#include <cstdint>

extern "C" {
#include "driver/gpio.h"  // ESP-IDF GPIO ISR service
#include "esp_intr_alloc.h"
#include "soc/gpio_reg.h"
}

namespace mae3 {

// Optional compile-time switch (define in platformio.ini if you want)
#ifndef MAE3_USE_FASTREAD
#define MAE3_USE_FASTREAD 1
#endif

#ifndef MAE3_TIME_SOURCE_CYCLE
#define MAE3_TIME_SOURCE_CYCLE 1
#endif

// Keep it in header to allow full inlining in ISR
inline bool fastRead(int pin) noexcept {
#if defined(ARDUINO_ARCH_ESP32) && (MAE3_USE_FASTREAD == 1)
  if (pin < 32) {
    return (REG_READ(GPIO_IN_REG) >> pin) & 0x1U;
  }
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 0x1U;
#else
  // Fallback: portable (slower) path
  return (digitalRead(pin) != 0);
#endif
}

#if MAE3_TIME_SOURCE_CYCLE
// Cycle counter (very low jitter). Requires fixed CPU freq (e.g., 240 MHz).
// If DFS/light-sleep is enabled, DO NOT use this path.
// for ESP.getCycleCount(), it needs #include <Arduino.h>
struct TimeStamp {
  static inline uint32_t nowTicks() noexcept {
    return ESP.getCycleCount();
  }  // 32-bit is fine for deltas
  static inline uint32_t toMicros(uint32_t cycles) noexcept {
    // 240 MHz -> 240 cycles/us. Adjust if using a different fixed CPU freq.
    return cycles / 240U;
  }
};
#else
struct TimeStamp {
  static inline uint32_t nowTicks() noexcept { return micros(); }
  static inline uint32_t toMicros(uint32_t us) noexcept { return us; }
};
#endif

// ---------- Types & Constants ----------
enum class Status : std::uint8_t {
  Ok = 0U,
  InvalidArg,
  NotInitialized,
  AlreadyInitialized
};
enum class LogicLevel : std::uint8_t { Low = 0U, High = 1U };

struct Constants final {
  static constexpr std::uint32_t kResolutionBits = 12U;
  static constexpr std::uint32_t kResolutionSteps =
      (1UL << kResolutionBits);                                      // 4096
  static constexpr std::uint32_t kMaxIndex = kResolutionSteps - 1U;  // 4095
  static constexpr std::float_t kMinTonUs = 0.95f;
  static constexpr std::uint32_t kMinCycleUs = 3892U;  // ~3.8 ms
  static constexpr std::uint32_t kMaxCycleUs = 4302U;  // ~4.3 ms
};

struct GpioConfig final {
  int pin;
  bool pullup;
  bool pulldown;  // note: Arduino INPUT_PULLDOWN not guaranteed for all pins;
                  // prefer external
  bool inverted;
};

class IEncoderObserver {
 public:
  virtual ~IEncoderObserver() = default;
  virtual void onPositionUpdate(std::uint8_t index, std::uint16_t position,
                                std::uint32_t tonUs, std::uint32_t toffUs) = 0;
};

// ---------- EdgeCapture (per channel, no heap) ----------
class EdgeCapture {
 public:
  EdgeCapture() = default;

  void reset(std::uint32_t now_ticks) noexcept {
    noInterrupts();
    last_edge_ticks_ = now_ticks;
    ton_ticks_ = 0U;
    toff_ticks_ = 0U;
    high_valid_ = false;
    low_valid_ = false;
    ready_ = false;
    interrupts();
  }

  // ISR-side: store delta only (no math/log/heap)
  inline void onEdgeISR(bool level, std::uint32_t now_ticks) noexcept {
    const std::uint32_t dt = now_ticks - last_edge_ticks_;
    last_edge_ticks_ = now_ticks;
    if (level) {
      toff_ticks_ = dt;
      low_valid_ = true;
    } else {
      ton_ticks_ = dt;
      high_valid_ = true;
    }
    if (high_valid_ && low_valid_) {
      ready_ = true;
      high_valid_ = false;
      low_valid_ = false;
    }
  }

  // Task-side: consume most recent full sample (non-blocking)
  bool tryConsume(std::uint32_t& ton_ticks,
                  std::uint32_t& toff_ticks) noexcept {
    noInterrupts();
    const bool rdy = ready_;
    if (rdy) {
      ton_ticks = ton_ticks_;
      toff_ticks = toff_ticks_;
      ready_ = false;
    }
    interrupts();
    return rdy;
  }

 private:
  volatile std::uint32_t last_edge_ticks_{0U};
  volatile std::uint32_t ton_ticks_{0U};
  volatile std::uint32_t toff_ticks_{0U};
  volatile bool high_valid_{false};
  volatile bool low_valid_{false};
  volatile bool ready_{false};
};

// ---------- Mae3Encoder (per channel) ----------
class Mae3Encoder {
 public:
  Mae3Encoder() = default;
  Mae3Encoder(std::uint8_t index, const GpioConfig& cfg)
      : index_{index}, cfg_{cfg} {}

  Status init() {
    if (inited_) {
      return Status::AlreadyInitialized;
    }
    pinMode(cfg_.pin, cfg_.pullup ? INPUT_PULLUP : INPUT);
    capture_.reset(mae3::TimeStamp::nowTicks());
    // Attach the same ISR function for all encoders; pass "this" as arg
    attachInterruptArg(digitalPinToInterrupt(cfg_.pin), &Mae3Encoder::isrShared,
                       this, CHANGE);
    disable();  // keep disabled until selected active
    inited_ = true;
    return Status::Ok;
  }

  Status deinit() {
    if (!inited_) {
      return Status::NotInitialized;
    }
    detachInterrupt(digitalPinToInterrupt(cfg_.pin));
    inited_ = false;
    enabled_ = false;
    return Status::Ok;
  }

  Status enable() {
    if (!inited_) {
      return Status::NotInitialized;
    }
    enabled_ = true;
    return Status::Ok;
  }
  Status disable() {
    if (!inited_) {
      return Status::NotInitialized;
    }
    enabled_ = false;
    return Status::Ok;
  }

  bool tryGetPosition(std::uint16_t& pos_out, std::uint32_t& tonUs,
                      std::uint32_t& toffUs) noexcept {
    std::uint32_t ton_raw{0U}, toff_raw{0U};
    if (!capture_.tryConsume(ton_raw, toff_raw)) {
      return false;
    }

    std::uint32_t ton_us = mae3::TimeStamp::toMicros(ton_raw);
    std::uint32_t toff_us = mae3::TimeStamp::toMicros(toff_raw);

    if (cfg_.inverted) {
      const std::uint32_t t = ton_us;
      ton_us = toff_us;
      toff_us = t;
    }
    pos_out = dutyToPosition(ton_us, toff_us);
    tonUs = ton_us;
    toffUs = toff_us;
    return true;
  }

  inline std::uint8_t index() const noexcept { return index_; }

 private:
  // --- Single ISR for all encoders (same function pointer) ---
  static void IRAM_ATTR isrShared(void* arg) {
    auto* self = static_cast<Mae3Encoder*>(arg);
    if (!self->enabled_) {
      return;
    }
    const bool level = fastRead(self->cfg_.pin);
    self->capture_.onEdgeISR(level, mae3::TimeStamp::nowTicks());
  }

  // --- Conversion (task-side) ---
  static std::uint16_t dutyToPosition(std::uint32_t ton_us,
                                      std::uint32_t toff_us) noexcept {
    const std::uint32_t period_us = ton_us + toff_us;
    if ((period_us < Constants::kMinCycleUs) ||
        (period_us > Constants::kMaxCycleUs) ||
        (ton_us < Constants::kMinTonUs)) {
      // Return last valid to avoid jitter when sample invalid
      return last_valid_pos_.load(std::memory_order_relaxed);
    }
    const uint32_t raw =
        (ton_us * (Constants::kResolutionSteps + 2U) + (period_us >> 1)) /
        period_us;

    const int pos = static_cast<int>(raw) - 1;
    const std::uint16_t clamped =
        (pos < 0) ? 0U
                  : (pos > static_cast<int>(Constants::kMaxIndex)
                         ? Constants::kMaxIndex
                         : static_cast<std::uint16_t>(pos));
    last_valid_pos_.store(clamped, std::memory_order_relaxed);
    return clamped;
  }

  static std::atomic<std::uint16_t> last_valid_pos_;

  std::uint8_t index_{0U};
  GpioConfig cfg_{};
  bool inited_{false};
  volatile bool enabled_{false};
  EdgeCapture capture_{};
};

inline std::atomic<std::uint16_t> Mae3Encoder::last_valid_pos_{0U};

// ---------- Manager (templated N; no heap; one-active-at-a-time) ----------
template <std::size_t N>
class EncoderManager {
 public:
  Status configure(const GpioConfig (&pins)[N]) {
    if (inited_) {
      return Status::AlreadyInitialized;
    }
    for (std::size_t i = 0U; i < N; ++i) {
      encs_[i] = Mae3Encoder(static_cast<std::uint8_t>(i),
                             pins[i]);  // RAII: stack object
      (void)encs_[i].init();
    }
    inited_ = true;
    return Status::Ok;
  }

  Mae3Encoder* get(std::size_t i) { return (i < N) ? &encs_[i] : nullptr; }

  // Per your original contract: only one encoder is active at any time.
  Status setActive(std::size_t i) {
    if (!inited_ || (i >= N)) {
      return Status::InvalidArg;
    }
    for (auto& e : encs_) {
      (void)e.disable();
    }
    active_ = &encs_[i];
    return active_->enable();
  }

  void attach(IEncoderObserver* obs) {
    for (auto& s : observers_) {
      if (s == nullptr) {
        s = obs;
        break;
      }
    }
  }

  void pollAndNotify() {
    if (active_ == nullptr) {
      return;
    }
    std::uint16_t pos{0U};
    std::uint32_t ton{0U};
    std::uint32_t toff{0U};
    if (active_->tryGetPosition(pos, ton, toff)) {
      for (auto* o : observers_) {
        if (o != nullptr) {
          o->onPositionUpdate(active_->index(), pos, ton, toff);
        }
      }
    }
  }

 private:
  bool inited_{false};
  Mae3Encoder encs_[N];
  Mae3Encoder* active_{nullptr};
  IEncoderObserver* observers_[N]{};  // fixed-capacity, no dynamic allocation
};

}  // namespace mae3

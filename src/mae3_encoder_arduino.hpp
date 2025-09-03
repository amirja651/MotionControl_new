#pragma once
#include <Arduino.h>

#include <atomic>
#include <cstdint>

namespace mae3 {

enum class EncoderId : std::uint8_t { Enc1 = 0U, Enc2, Enc3, Enc4 };
enum class Status : std::uint8_t {
  Ok = 0U,
  InvalidArg,
  NotInitialized,
  AlreadyInitialized
};
enum class LogicLevel : std::uint8_t { Low = 0U, High = 1U };

struct Constants final {
  static constexpr std::uint32_t kResolutionBits = 12U;
  static constexpr std::uint32_t kResolutionSteps = (1UL << kResolutionBits);
  static constexpr std::uint32_t kMaxIndex = kResolutionSteps - 1U;
  static constexpr std::uint32_t kMinTonUs = 1U;
  static constexpr std::uint32_t kMinCycleUs = 3500U;
  static constexpr std::uint32_t kMaxCycleUs = 4300U;
};

struct GpioConfig final {
  int pin;
  bool pullup;
  bool pulldown;
  bool inverted;
};

class IEncoderObserver {
 public:
  virtual ~IEncoderObserver() = default;
  virtual void onPositionUpdate(EncoderId id, std::uint16_t position) = 0;
};

class EdgeCapture {
 public:
  EdgeCapture() = default;
  void reset(std::uint32_t now) noexcept {
    noInterrupts();
    last_edge_us_ = now;
    ton_us_ = 0U;
    toff_us_ = 0U;
    hv_ = false;
    lv_ = false;
    ready_ = false;
    interrupts();
  }
  void onEdgeISR(bool level, std::uint32_t now) noexcept {
    const std::uint32_t dt = now - last_edge_us_;
    last_edge_us_ = now;
    if (level) {
      toff_us_ = dt;
      lv_ = true;
    } else {
      ton_us_ = dt;
      hv_ = true;
    }
    if (hv_ && lv_) {
      ready_ = true;
      hv_ = lv_ = false;
    }
  }
  bool tryConsume(std::uint32_t& ton, std::uint32_t& toff) noexcept {
    noInterrupts();
    const bool rdy = ready_;
    if (rdy) {
      ton = ton_us_;
      toff = toff_us_;
      ready_ = false;
    }
    interrupts();
    return rdy;
  }

 private:
  volatile std::uint32_t last_edge_us_{0U};
  volatile std::uint32_t ton_us_{0U};
  volatile std::uint32_t toff_us_{0U};
  volatile bool hv_{false}, lv_{false}, ready_{false};
};

class Mae3Encoder {
 public:
  explicit Mae3Encoder(EncoderId id, const GpioConfig& cfg)
      : id_{id}, cfg_{cfg} {}
  Mae3Encoder(const Mae3Encoder&) = delete;
  Mae3Encoder& operator=(const Mae3Encoder&) = delete;  // amir

  Status init() {
    if (inited_) {
      return Status::AlreadyInitialized;
    }
    pinMode(cfg_.pin, INPUT);
    if (cfg_.pullup) {
      pinMode(cfg_.pin, INPUT_PULLUP);
    }
    // (ESP32 Arduino lacks INPUT_PULLDOWN on all pins reliably; wire extern
    // pull-down if needed)
    capture_.reset(micros());
    attachInterruptArg(digitalPinToInterrupt(cfg_.pin), &Mae3Encoder::isrThunk,
                       this, CHANGE);
    disable();
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

  bool tryGetPosition(std::uint16_t& pos) {
    std::uint32_t ton{0U}, toff{0U};
    if (!capture_.tryConsume(ton, toff)) {
      return false;
    }
    if (cfg_.inverted) {
      auto t = ton;
      ton = toff;
      toff = t;
    }
    pos = dutyToPosition(ton, toff);
    return true;
  }

  inline EncoderId id() const noexcept { return id_; }

 private:
  static void IRAM_ATTR isrThunk(void* arg) {
    auto* self = static_cast<Mae3Encoder*>(arg);
    if (!self->enabled_) {
      return;
    }
    const bool level = digitalRead(self->cfg_.pin);
    self->capture_.onEdgeISR(level, micros());
  }

  static std::uint16_t dutyToPosition(std::uint32_t ton,
                                      std::uint32_t toff) noexcept {
    const std::uint32_t period = ton + toff;
    if ((period < Constants::kMinCycleUs) ||
        (period > Constants::kMaxCycleUs) || (ton < Constants::kMinTonUs)) {
      return last_valid_pos_.load(std::memory_order_relaxed);
    }
    const std::uint32_t raw =
        (ton * (Constants::kResolutionSteps + 1UL)) / period;
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

  EncoderId id_;
  GpioConfig cfg_;
  bool inited_{false};
  volatile bool enabled_{false};
  EdgeCapture capture_{};
};

inline std::atomic<std::uint16_t> Mae3Encoder::last_valid_pos_{0U};

// Manager (Singleton) for 4 encoders, one active
class EncoderManager {
 public:
  static EncoderManager& instance() {
    static EncoderManager m;
    return m;
  }
  Status configure(const GpioConfig (&pins)[4]) {
    if (inited_) {
      return Status::AlreadyInitialized;
    }
    for (std::uint8_t i = 0U; i < 4U; ++i) {
      encs_[i] = Mae3Encoder(static_cast<EncoderId>(i), pins[i]);
      (void)encs_[i].init();
    }
    inited_ = true;
    return Status::Ok;
  }
  Mae3Encoder* get(EncoderId id) {
    return inited_ ? &encs_[static_cast<std::uint8_t>(id)] : nullptr;
  }
  Status setActive(EncoderId id) {
    if (!inited_) {
      return Status::NotInitialized;
    }
    for (auto& e : encs_) {
      (void)e.disable();
    }
    active_ = &encs_[static_cast<std::uint8_t>(id)];
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
    std::uint16_t p{0U};
    if (active_->tryGetPosition(p)) {
      for (auto* o : observers_) {
        if (o != nullptr) {
          o->onPositionUpdate(active_->id(), p);
        }
      }
    }
  }

 private:
  EncoderManager() = default;
  bool inited_{false};
  Mae3Encoder encs_[4] = {
      Mae3Encoder(EncoderId::Enc1, {0, false, false, false}),
      Mae3Encoder(EncoderId::Enc2, {0, false, false, false}),
      Mae3Encoder(EncoderId::Enc3, {0, false, false, false}),
      Mae3Encoder(EncoderId::Enc4, {0, false, false, false})};
  Mae3Encoder* active_{nullptr};
  IEncoderObserver* observers_[4]{};
};

}  // namespace mae3

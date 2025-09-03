// Layer: mae3_encoder.cpp — Driver/Manager implementation + ISR + no heap

#include "mae3_encoder.hpp"

#include <new>

#include "esp_check.h"
#include "esp_intr_alloc.h"

namespace mae3 {

std::atomic<std::uint16_t> Mae3Encoder::last_valid_pos_{0U};

// ---- Mae3Encoder ----

Status Mae3Encoder::init() {
  if (inited_) {
    return Status::AlreadyInitialized;
  }

  gpio_config_t io{};
  io.intr_type = GPIO_INTR_ANYEDGE;
  io.mode = GPIO_MODE_INPUT;
  io.pin_bit_mask = (1ULL << static_cast<std::uint32_t>(cfg_.pin));
  io.pull_down_en =
      cfg_.pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = cfg_.pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  if (gpio_config(&io) != ESP_OK) {
    return Status::InvalidArg;
  }

  // Install shared ISR service (once globally is fine; idempotent)
  static bool isr_service_installed = false;
  if (!isr_service_installed) {
    ESP_ERROR_CHECK(gpio_install_isr_service(
        ESP_INTR_FLAG_IRAM));  // allocates once, not in RT path
    isr_service_installed = true;
  }

  // Attach ISR but keep disabled until enable()
  capture_.reset(static_cast<std::uint32_t>(esp_timer_get_time()));
  ESP_ERROR_CHECK(
      gpio_isr_handler_add(cfg_.pin, &Mae3Encoder::isrTrampoline, this));
  ESP_ERROR_CHECK(gpio_intr_disable(cfg_.pin));

  inited_ = true;
  return Status::Ok;
}

Status Mae3Encoder::deinit() {
  if (!inited_) {
    return Status::NotInitialized;
  }
  ESP_ERROR_CHECK(gpio_isr_handler_remove(cfg_.pin));
  inited_ = false;
  enabled_ = false;
  return Status::Ok;
}

Status Mae3Encoder::enable() {
  if (!inited_) {
    return Status::NotInitialized;
  }
  capture_.reset(static_cast<std::uint32_t>(esp_timer_get_time()));
  ESP_ERROR_CHECK(gpio_intr_enable(cfg_.pin));
  enabled_ = true;
  return Status::Ok;
}

Status Mae3Encoder::disable() {
  if (!inited_) {
    return Status::NotInitialized;
  }
  ESP_ERROR_CHECK(gpio_intr_disable(cfg_.pin));
  enabled_ = false;
  return Status::Ok;
}

bool Mae3Encoder::tryGetPosition(std::uint16_t& position) {
  std::uint32_t ton{0U}, toff{0U};
  if (!capture_.tryConsumeSample(ton, toff)) {
    return false;
  }

  // Inversion support (optional wiring)
  if (cfg_.inverted) {
    const std::uint32_t t = ton;
    ton = toff;
    toff = t;
  }
  position = dutyToPosition(ton, toff);
  return true;
}

void IRAM_ATTR Mae3Encoder::isrTrampoline(void* arg) {
  auto* self = static_cast<Mae3Encoder*>(arg);
  const bool lvl = gpio_get_level(self->cfg_.pin);  // register read, IRAM-safe
  const std::uint32_t now = static_cast<std::uint32_t>(esp_timer_get_time());
  self->capture_.onEdgeISR(lvl ? LogicLevel::High : LogicLevel::Low, now);
}

// ---- EncoderManager ----

EncoderManager& EncoderManager::instance() {
  static EncoderManager inst{};
  return inst;
}

Status EncoderManager::configure(const std::array<GpioConfig, 4U>& pins) {
  if (encs_ == nullptr) {
    // In-place construct without heap
    encs_ = reinterpret_cast<std::array<Mae3Encoder, 4U>*>(storage_);
    new (&(*encs_)[0]) Mae3Encoder(EncoderId::Enc1, pins[0]);
    new (&(*encs_)[1]) Mae3Encoder(EncoderId::Enc2, pins[1]);
    new (&(*encs_)[2]) Mae3Encoder(EncoderId::Enc3, pins[2]);
    new (&(*encs_)[3]) Mae3Encoder(EncoderId::Enc4, pins[3]);
    for (auto& e : *encs_) {
      (void)e.init();
    }
    active_ = nullptr;
    return Status::Ok;
  }
  return Status::AlreadyInitialized;
}

Mae3Encoder* EncoderManager::get(EncoderId id) {
  if (encs_ == nullptr) {
    return nullptr;
  }
  return &(*encs_)[static_cast<std::size_t>(id)];
}

Status EncoderManager::setActive(EncoderId id) {
  if (encs_ == nullptr) {
    return Status::NotInitialized;
  }
  // Disable all, enable selected
  for (auto& e : *encs_) {
    (void)e.disable();
  }
  active_ = &(*encs_)[static_cast<std::size_t>(id)];
  return active_->enable();
}

Status EncoderManager::attach(IEncoderObserver* obs) {
  if (obs == nullptr) {
    return Status::InvalidArg;
  }
  for (auto& slot : observers_) {
    if (slot == nullptr) {
      slot = obs;
      return Status::Ok;
    }
  }
  return Status::InvalidArg;  // full
}

Status EncoderManager::detach(IEncoderObserver* obs) {
  if (obs == nullptr) {
    return Status::InvalidArg;
  }
  for (auto& slot : observers_) {
    if (slot == obs) {
      slot = nullptr;
      return Status::Ok;
    }
  }
  return Status::InvalidArg;
}

void EncoderManager::pollAndNotify() {
  if (active_ == nullptr) {
    return;
  }
  std::uint16_t pos{0U};
  if (active_->tryGetPosition(pos)) {
    for (auto* o : observers_) {
      if (o != nullptr) {
        o->onPositionUpdate(active_->id(), pos);
      }
    }
  }
}

}  // namespace mae3

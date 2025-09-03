#include <Arduino.h>

#include "mae3_encoder_arduino.hpp"

using namespace mae3;

class LoggerObserver final : public IEncoderObserver {
 public:
  void onPositionUpdate(EncoderId id, std::uint16_t position) override {
    static std::uint32_t c{0U};
    if ((++c % 50U) == 0U) {
      Serial.printf("Enc[%u] pos=%u\r\n", static_cast<unsigned>(id),
                    static_cast<unsigned>(position));
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);

  GpioConfig pins[4] = {
      {4, true, false, false},
      {5, true, false, false},
      {18, true, false, false},
      {19, true, false, false},
  };

  auto& mgr = EncoderManager::instance();
  (void)mgr.configure(pins);

  static LoggerObserver logObs;
  mgr.attach(&logObs);

  (void)mgr.setActive(EncoderId::Enc1);
}

void loop() {
  // Non-blocking polling; keep ISR tiny.
  EncoderManager::instance().pollAndNotify();

  // Example: rotate active every 5s
  static uint32_t t0 = millis();
  static EncoderId ids[] = {EncoderId::Enc1, EncoderId::Enc2, EncoderId::Enc3,
                            EncoderId::Enc4};
  static size_t idx = 0U;
  if (millis() - t0 > 5000U) {
    t0 = millis();
    idx = (idx + 1U) % 4U;
    (void)EncoderManager::instance().setActive(ids[idx]);
  }
  // Yield a bit
  delay(2);
}

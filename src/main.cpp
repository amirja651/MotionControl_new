#include <Arduino.h>

#include "mae3_encoder_arduino.hpp"
using namespace mae3;

class LoggerObserver final : public IEncoderObserver {
 public:
  void onPositionUpdate(std::uint8_t index, std::uint16_t position,
                        std::uint32_t tonUs, std::uint32_t toffUs) override {
    static std::uint32_t cnt{0U};
    if ((++cnt % 50U) == 0U) {
      const std::uint32_t period = tonUs + toffUs;
      Serial.printf(
          "Enc[%u] pos=%u ton=%uus period=%uus\n", static_cast<unsigned>(index),
          static_cast<unsigned>(position), static_cast<unsigned>(tonUs),
          static_cast<unsigned>(period));
    }
  }
};

// Choose N freely (1..N)
constexpr std::size_t kN = 4U;
EncoderManager<kN> manager;

void setup() {
  Serial.begin(115200);
  GpioConfig pins[kN] = {
      {36, true, false, false},
      {39, true, false, false},
      {34, true, false, false},
      {35, true, false, false},
  };
  (void)manager.configure(pins);

  static LoggerObserver obs{};
  manager.attach(&obs);

  // Contract: one active at a time
  (void)manager.setActive(0U);
}

void loop() {
  manager.pollAndNotify();

  // Rotate active encoder every 5s (example policy)
  static std::uint32_t t0 = millis();
  static std::size_t idx = 0U;
  if (millis() - t0 > 5000U) {
    t0 = millis();
    idx = (idx + 1U) % kN;
    (void)manager.setActive(idx);
  }
  delay(2);
}

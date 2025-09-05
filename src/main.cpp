#include <Arduino.h>

#include "TMC5160Module.hpp"

// ESP32 VSPI pins (typical; adjust to your board wiring)
constexpr uint8_t PIN_SCK = 18U;
constexpr uint8_t PIN_MISO = 19U;
constexpr uint8_t PIN_MOSI = 23U;

// Example STEP/DIR/CS/EN pins per driver (adjust to your wiring)
constexpr uint8_t M1_STEP = 25U;
constexpr uint8_t M1_DIR = 26U;
constexpr uint8_t M1_CS = 5U;
constexpr int8_t M1_EN = 27;  // -1 if not used

constexpr uint8_t M2_STEP = 14U;
constexpr uint8_t M2_DIR = 12U;
constexpr uint8_t M2_CS = 4U;
constexpr int8_t M2_EN = 13;

SPIClass SPIbus(VSPI);  // Shared bus

// Rotational motor config: 0.01 .. 359.95 degrees
MotorConfig cfg_rot{/*step*/ M1_STEP,
                    /*dir*/ M1_DIR,
                    /*cs*/ M1_CS,
                    /*en*/ M1_EN,
                    /*enPol*/ EnablePolarity::ActiveLow,
                    /*inv*/ false,
                    /*kind*/ MotorKind::Rotational,
                    /*name*/ "ROT-A",
                    /*steps_per_rev*/ 200U,
                    /*microsteps*/ 16U,
                    /*um_per_rev*/ 200U,  // not used for rotational, kept for completeness
                    /*hold_mA*/ 300U,
                    /*run_mA*/ 900U,
                    /*r_sense*/ 0.075f,
                    /*min_user*/ 0.01,
                    /*max_user*/ 359.95,
                    /*max_speed_sps*/ 4000.0f,
                    /*max_accel*/ 8000.0f};

// Linear motor config: -3000 .. +3000 µm ; spec says 1 rev = 200 µm
MotorConfig cfg_lin{/*step*/ M2_STEP,
                    /*dir*/ M2_DIR,
                    /*cs*/ M2_CS,
                    /*en*/ M2_EN,
                    /*enPol*/ EnablePolarity::ActiveLow,
                    /*inv*/ false,
                    /*kind*/ MotorKind::Linear,
                    /*name*/ "LIN-Z",
                    /*steps_per_rev*/ 200U,
                    /*microsteps*/ 32U,
                    /*um_per_rev*/ 200U,  // per spec
                    /*hold_mA*/ 300U,
                    /*run_mA*/ 1000U,
                    /*r_sense*/ 0.075f,
                    /*min_user*/ -3000.0,
                    /*max_user*/ +3000.0,
                    /*max_speed_sps*/ 5000.0f,
                    /*max_accel*/ 12000.0f};

TMC5160Motor motor_rot(cfg_rot, SPIbus);
TMC5160Motor motor_lin(cfg_lin, SPIbus);
TMC5160BusManager bus(SPIbus);

void setup() {
  Serial.begin(115200);
  delay(50);

  bus.addMotor(&motor_rot);
  bus.addMotor(&motor_lin);

  const bool ok = bus.begin(PIN_SCK, PIN_MISO, PIN_MOSI, 2000000UL);
  Serial.printf("Bus begin: %s\r\n", ok ? "OK" : "FAIL");

  // Enable drivers after init (safe current pre-configured)
  motor_rot.driverEnable(true);
  motor_lin.driverEnable(true);

  // Probe which motors are present (SPI test)
  ProbeResult res[4]{};
  const size_t n = bus.probeAll(res, 4U);
  for (size_t i = 0; i < n; ++i) {
    Serial.printf("Probe cs=%u name=%s -> %s\r\n", res[i].cs_pin, (res[i].name != nullptr ? res[i].name : "(null)"),
                  res[i].connected ? "CONNECTED" : "NOT FOUND");
  }

  // Example commands (non-blocking)
  (void)motor_rot.moveToUser(90.0);     // degrees (within 0.01..359.95)
  (void)motor_lin.moveToUser(+1500.0);  // micrometers (within -3000..+3000)
}

void loop() {
  // Non-blocking service (ensure this runs fast — no delays)
  bus.serviceAll();

  // Example: constant-speed jog on the linear axis after reaching target
  static bool jog_started = false;
  if (!jog_started && fabs(motor_lin.currentPositionUser() - 1500.0) < 5.0) {
    motor_lin.setSpeedUser(+50.0);  // +50 µm/s
    jog_started = true;
  }

  // Pet the watchdog here if you enabled one in your system
}

#if (BOARD_TYPE == 1)
    #include "MiniLinkSlave.h"

    #include <Arduino.h>
    #include <Preferences.h>

// Every 200 µm is equivalent to one full revolution (360°) of the encoder
constexpr float STEP_UNIT_UM      = 1.0f;    // Size of each step in µm (e.g. 1 µm or 0.5 µm depending on the resolution)
constexpr float REV_DISTANCE_UM   = 200.0f;  // One full revolution of the motor in µm
constexpr float TOLERANCE_UM      = 0.03f;   // Tolerance allowed for "boundary" detection
constexpr float FULL_ROTATION_DEG = 360.0f;

//---------- Used in onHomingComplete ----
// Define globals for chaining
static volatile uint8_t  gCurrentMotor   = 0;
static volatile double_t gTargetPosition = 0.0f;

// --- Chaining state (Rapid -> Touch) ---
static volatile int32_t gTouchFinalSteps = 0;

// Runtime state
volatile bool gVoltageDrop = false;

// To display the time interval between K/Js
static uint32_t gLastKPressMs = 0;

// ---------- Project constants ----------
static constexpr uint8_t  kMotorCount         = 4;
static constexpr uint8_t  kLinearMotorIndex   = 0;        // M1 (index 0)
static constexpr uint8_t  kFirstRotaryIndex   = 1;        // M2..M4
static constexpr double_t kDefaultLeadPitchUm = 200.0;    // default: 0.2 mm/rev = 200 µm
static constexpr int32_t  SPI_CLOCK           = 1000000;  // 1MHz SPI clock
static constexpr uint32_t kSerialBaud         = 115200;
static constexpr uint32_t kWatchdogSeconds    = 15;

// --- Rotary touch-window and speed profiles ---
static constexpr int32_t  kTouchWindowStep = 10;     // final window near target in steps
static constexpr double_t kTouchWindowDeg  = 1.0;    // final window near target in degrees
static constexpr float    kFastMaxSpeed    = 16000;  // steps/s for Rapid
static constexpr float    kFastAccel       = 24000;  // steps/s^2 for Rapid
static constexpr float    kTouchMaxSpeed   = 600;    // steps/s for Touch
static constexpr float    kTouchAccel      = 1200;   // steps/s^2 for Touch

// ---------- Persistence (NVS/Preferences) ----------
Preferences gPrefs;  // namespace: "motion"

MiniLinkSlave linkSlave;

void setup()
{
    Serial.begin(kSerialBaud);
    esp_log_level_set("*", ESP_LOG_INFO);
    delay(1000);

    linkSlave.begin2();
    Serial.println("SPI Slave ready.");
}

void loop()
{
    linkSlave.serviceOnce();  // blocks until a frame arrives
}
#endif
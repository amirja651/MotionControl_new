#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <bitset>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>
#include <functional>  // For callback support
#include <unordered_map>
#include <vector>

// Physical parameters
#define PIXEL_SIZE_UM      5.2f  // Size of each pixel in the camera (micrometers)
#define PIXEL_SIZE_MM      (PIXEL_SIZE_UM * 1e-3f)
#define L_MM               195.0f  // Distance from mirror to camera in millimeters (can be measured accurately)
#define MICROSTEP          16      // Or 256
#define STEPS_PER_REV      200     // Motor step = 1.8° = 200 steps
#define ENCODER_RESOLUTION 4096    // Encoder 12 bits
#define GEAR_RATIO         1.0f    // If there is no gear ratio = 1
// Calculate the number of actual pulses in one complete motor revolution
#define TOTAL_PULSES_PER_REV (STEPS_PER_REV * MICROSTEP)

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    volatile int64_t width_high;
    volatile int64_t width_low;
    volatile int64_t width_interval;

    int32_t lap_id;
    int32_t position_pulse;

    float position_degrees;
    float position_mm;
    float total_mm;

    Direction direction;
};

static constexpr int8_t MAX_LAPS = 20;

struct RPulse
{
    volatile int64_t low;
    volatile int64_t high;
    volatile int64_t pulse_Interval;
    volatile int64_t duration_us_count;
};

class MAE3Encoder
{
public:
    MAE3Encoder(uint8_t signalPin, uint8_t encoderId);
    ~MAE3Encoder();

    bool begin();

    void enable();
    void disable();

    bool isEnabled() const;
    bool isDisabled() const;

    void reset();
    void processPWM(bool print = false);

    int32_t umToPulses(float um);
    int32_t degreesToPulses(float degrees);
    float   pulsesToUm(float pulses);

    bool isStopped(int64_t threshold_us = 500000 /* 500ms */) const;
    void setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb);
    void diagnoseEncoderSignals();

    float calculateMirrorAngle(int currentPixel, int referencePixel);
    float calculateDistanceToMirror(int currentPixel, int referencePixel, float mirrorAngleDeg);
    float calculateEncoderAngle(int currentCount, int previousCount, int encoderResolution = 4096, float gearRatio = 1.0);

    float pixelToMirrorAngle(int deltaPixel);
    int   mirrorAngleToPulses(float mirrorAngleDeg);
    int   pixelToPulses(int deltaPixel);
    int   mirrorAngleToPulses(float mirrorAngleDeg, int microstep, int stepsPerRev = 200, float gearRatio = 1.0);
    int   encoderToPulses(int encoderValue, int microstep, int stepsPerRev = 200, int encoderResolution = 4096);

    EncoderState getState() const
    {
        return _state;
    }

private:
    // Pin assignments
    const uint8_t    _signalPin;
    const uint8_t    _encoderId;
    EncoderState     _state;
    RPulse           _r_pulse;
    int64_t          _lastPulseTime;
    volatile int64_t _lastFallingEdgeTime;
    volatile int64_t _lastRisingEdgeTime;
    volatile int64_t _currentRisingEdge;
    volatile int64_t _lowStart;
    volatile int64_t _lowEnd;
    volatile bool    _enabled;
    volatile bool    _dataReady;
    bool             _initialized;
    int32_t          _last_pulse;

    // Maximum number of encoders supported
    static constexpr float  LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm
    static constexpr size_t PULSE_BUFFER_SIZE   = 64;    // Pulse width ring buffers

    static MAE3Encoder* _encoderInstances[4];  // Static array to store encoder instances for interrupt handling

    std::array<int64_t, PULSE_BUFFER_SIZE> _width_l_buffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> _width_h_buffer{};
    size_t                                 _pulseBufferIndex;

    std::function<void(const EncoderState&)> onPulseUpdated;  // NEW: callback support

    void attachInterruptHandler();
    void detachInterruptHandler();

    void IRAM_ATTR processInterrupt();

    int64_t get_median_width_high() const;
    int64_t get_median_width_low() const;

    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();
};

#endif  // MAE3_ENCODER2_H
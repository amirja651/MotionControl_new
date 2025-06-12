#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <bitset>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>
#include <functional>  // For callback support

// #define DEBUG_ENCODER true

// Maximum number of encoders supported
const uint8_t MAX_ENCODERS = 4;
const int8_t  MAX_LAPS     = 20;
const int8_t  LAPS_OFFSET  = 10;

const int64_t DIR_THRESHOLD       = 2;     // For example, if the difference is more than 2 pulses → change direction
const int64_t FULL_SCALE          = 4096;  // 0..4095
const int64_t HIGH_WRAP_THRESHOLD = 1000;
const int64_t LOW_WRAP_THRESHOLD  = -1000;

// Linear motion constants
const float LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    volatile int32_t current_pulse;  // Current pulse value
    volatile int64_t width_high;     // High pulse width (rising to falling)
    volatile int64_t width_low;      // Low pulse width (falling to rising)
    Direction        direction;      // Current direction of rotation
};

struct LapState
{
    volatile int32_t id;
    uint32_t         period[MAX_LAPS]       = {0};
    int64_t          period_sum[MAX_LAPS]   = {0};
    uint32_t         period_count[MAX_LAPS] = {0};
};

struct EncoderContext
{
    int32_t current_pulse;

    int32_t  lap_id;
    uint32_t lap_period;

    // float average_period;

    float position_degrees;  //  Rotary motors

    float position_mm;  //  Linear motors
    float total_travel_mm;
    float total_travel_um;

    const char* direction;
};

struct RPulse
{
    volatile int64_t low  = 0;
    volatile int64_t high = 0;
};

class MAE3Encoder
{
public:
    MAE3Encoder(uint8_t signalPin, uint8_t encoderId);
    ~MAE3Encoder();

    bool begin();
    void enable();
    void disable();
    void reset();
    void processPWM();

    inline bool isEnabled() const
    {
        return enabled;
    }

    inline bool isDisabled() const
    {
        return !enabled;
    }

    EncoderContext& getEncoderContext() const
    {
        portENTER_CRITICAL(&mux);
        encoderContext.current_pulse = state.current_pulse;
        encoderContext.direction     = state.direction == Direction::UNKNOWN     ? "   "
                                       : state.direction == Direction::CLOCKWISE ? " CW"
                                                                                 : "CCW";
        encoderContext.lap_id        = lap.id;
        encoderContext.lap_period    = lap.period[lap.id + LAPS_OFFSET];

        encoderContext.position_degrees = state.current_pulse * (360.0f / FULL_SCALE);
        encoderContext.position_mm      = encoderContext.current_pulse * (LEAD_SCREW_PITCH_MM / FULL_SCALE);
        encoderContext.total_travel_mm  = (lap.id * LEAD_SCREW_PITCH_MM) + encoderContext.position_mm;
        encoderContext.total_travel_um  = encoderContext.total_travel_mm * 1000.0f;
        portEXIT_CRITICAL(&mux);
        return encoderContext;
    }

    // Converts um to pulses
    inline float umToPulses(float um)
    {
        float mm            = um / 1000.0f;
        float pulses_per_mm = FULL_SCALE / LEAD_SCREW_PITCH_MM;
        return mm * pulses_per_mm;
    }

    // Converts pulses to um
    inline float pulsesToUm(float pulses)
    {
        float mm = pulses * (LEAD_SCREW_PITCH_MM / FULL_SCALE);
        return mm * 1000.0f;
    }

    bool isStopped(int64_t threshold_us = 500000 /* 500ms */) const
    {
        return (esp_timer_get_time() - lastPulseTime) > threshold_us;
    }

    void setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb)
    {
        onPulseUpdated = cb;
    }

private:
    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState           state;
    LapState               lap;
    mutable EncoderContext encoderContext;

    volatile bool enabled = false;

    RPulse r_pulse;

    // Filtering and timing
    int64_t          lastPulseTime       = 0;
    volatile int64_t lastFallingEdgeTime = 0;
    volatile int64_t lastRisingEdgeTime  = 0;

    // Interrupt handling
    volatile bool newPulseAvailable = false;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    volatile bool bufferUpdated = false;

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* encoderInstances[MAX_ENCODERS];

    // --- Pulse width ring buffers ---
    static constexpr size_t PULSE_BUFFER_SIZE = 5;

    std::array<int64_t, PULSE_BUFFER_SIZE> width_l_buffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> width_h_buffer{};

    size_t pulseBufferIndex = 0;

    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    std::function<void(const EncoderState&)> onPulseUpdated;  // NEW: callback support

    int32_t last_pulse;
    bool    initialized;

    void processInterrupt();
    void attachInterruptHandler();
    void detachInterruptHandler();

    int64_t get_median_width_high() const;
    int64_t get_median_width_low() const;

    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    inline void setPeriod(int32_t lapIndex, int64_t period, bool reset_count = false)
    {
        lap.period[lapIndex + LAPS_OFFSET] = static_cast<int32_t>(period);

        if (reset_count)
        {
            lap.period_sum[lap.id + LAPS_OFFSET]   = static_cast<int32_t>(period);
            lap.period_count[lap.id + LAPS_OFFSET] = 1;
        }
        else
        {
            lap.period_sum[lap.id + LAPS_OFFSET] += static_cast<int32_t>(period);
            lap.period_count[lap.id + LAPS_OFFSET]++;
        }
    }

    void resetAllPeriods()
    {
        lap.id = 0;
        memset(lap.period, 0, sizeof(lap.period));
        memset(lap.period_sum, 0, sizeof(lap.period_sum));
        memset(lap.period_count, 0, sizeof(lap.period_count));
    }
};

#endif  // MAE3_ENCODER2_H
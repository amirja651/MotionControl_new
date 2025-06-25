#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <algorithm>
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
        return _enabled;
    }

    inline bool isDisabled() const
    {
        return !_enabled;
    }

    EncoderContext& getEncoderContext() const
    {
        portENTER_CRITICAL(&classMux);
        _encoderContext.current_pulse = _state.current_pulse;
        _encoderContext.direction     = _state.direction == Direction::UNKNOWN     ? "   "
                                        : _state.direction == Direction::CLOCKWISE ? " CW"
                                                                                   : "CCW";
        _encoderContext.lap_id        = _lap.id;
        _encoderContext.lap_period    = _lap.period[_lap.id + LAPS_OFFSET];

        _encoderContext.position_degrees = _state.current_pulse * (360.0f / FULL_SCALE);
        _encoderContext.position_mm      = _encoderContext.current_pulse * (LEAD_SCREW_PITCH_MM / FULL_SCALE);
        _encoderContext.total_travel_mm  = (_lap.id * LEAD_SCREW_PITCH_MM) + _encoderContext.position_mm;
        _encoderContext.total_travel_um  = _encoderContext.total_travel_mm * 1000.0f;
        portEXIT_CRITICAL(&classMux);
        return _encoderContext;
    }

    // Converts um to pulses
    inline int32_t umToPulses(float um)
    {
        float mm            = um / 1000.0f;
        float pulses_per_mm = FULL_SCALE / LEAD_SCREW_PITCH_MM;
        return static_cast<int32_t>(mm * pulses_per_mm);
    }

    // Converts degrees to pulses for rotary motors
    inline int32_t degreesToPulses(float degrees)
    {
        float pulses_per_degree = FULL_SCALE / 360.0f;
        return static_cast<int32_t>(degrees * pulses_per_degree);
    }

    // Converts pulses to um
    inline float pulsesToUm(float pulses)
    {
        float mm = pulses * (LEAD_SCREW_PITCH_MM / FULL_SCALE);
        return mm * 1000.0f;
    }

    bool isStopped(int64_t threshold_us = 500000 /* 500ms */) const
    {
        return (esp_timer_get_time() - _lastPulseTime) > threshold_us;
    }

    void setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb)
    {
        onPulseUpdated = cb;
    }

private:
    // Pin assignments
    const uint8_t _signalPin;
    const uint8_t _encoderId;

    // State management
    EncoderState           _state;
    LapState               _lap;
    mutable EncoderContext _encoderContext;

    volatile bool _enabled = false;

    RPulse _r_pulse;

    // Filtering and timing
    int64_t          _lastPulseTime       = 0;
    volatile int64_t _lastFallingEdgeTime = 0;
    volatile int64_t _lastRisingEdgeTime  = 0;

    // Interrupt handling
    volatile bool _newPulseAvailable = false;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    volatile bool _bufferUpdated = false;

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* _encoderInstances[MAX_ENCODERS];

    // --- Pulse width ring buffers ---
    static constexpr size_t _PULSE_BUFFER_SIZE = 5;

    std::array<int64_t, _PULSE_BUFFER_SIZE> _width_l_buffer{};
    std::array<int64_t, _PULSE_BUFFER_SIZE> _width_h_buffer{};

    size_t _pulseBufferIndex = 0;

    // mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    static portMUX_TYPE classMux;

    std::function<void(const EncoderState&)> onPulseUpdated;  // NEW: callback support

    int32_t _last_pulse;
    bool    _initialized;

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
        _lap.period[lapIndex + LAPS_OFFSET] = static_cast<int32_t>(period);

        if (reset_count)
        {
            _lap.period_sum[_lap.id + LAPS_OFFSET]   = static_cast<int32_t>(period);
            _lap.period_count[_lap.id + LAPS_OFFSET] = 1;
        }
        else
        {
            _lap.period_sum[_lap.id + LAPS_OFFSET] += static_cast<int32_t>(period);
            _lap.period_count[_lap.id + LAPS_OFFSET]++;
        }
    }

    void resetAllPeriods()
    {
        _lap.id = 0;
        memset(_lap.period, 0, sizeof(_lap.period));
        memset(_lap.period_sum, 0, sizeof(_lap.period_sum));
        memset(_lap.period_count, 0, sizeof(_lap.period_count));
    }
};

#endif  // MAE3_ENCODER2_H
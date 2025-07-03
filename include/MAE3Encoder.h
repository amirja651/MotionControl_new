#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <bitset>
#include <cstdlib>  // For abs with uint32_t
#include <esp_timer.h>
#include <functional>  // For callback support

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

static constexpr int8_t MAX_LAPS = 20;

struct LapState
{
    volatile int32_t id;
    uint32_t         period[MAX_LAPS]       = {0};
    int64_t          period_sum[MAX_LAPS]   = {0};
    uint32_t         period_count[MAX_LAPS] = {0};
};

struct EncoderContext
{
    int32_t  current_pulse;
    int32_t  lap_id;
    uint32_t lap_period;

    float position_degrees;  //  Rotary motors
    float position_mm;       //  Linear motors
    float total_travel_mm;
    float total_travel_um;

    const char* direction;
};

struct RPulse
{
    volatile int64_t low;
    volatile int64_t high;
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
    void processPWM();

    static int getNumberOfInterruptsAttached(uint8_t encoderId)
    {
        return _interruptsAttached[encoderId];
    }
    static int getNumberOfInterruptsDetached(uint8_t encoderId)
    {
        return _interruptsDetached[encoderId];
    }
    static int getNumberOfProcessInterrupts(uint8_t encoderId)
    {
        return _processInterrupt[encoderId];
    }

    EncoderContext& getEncoderContext() const;

    int32_t umToPulses(float um);
    int32_t degreesToPulses(float degrees);
    float   pulsesToUm(float pulses);

    bool isStopped(int64_t threshold_us = 500000 /* 500ms */) const;
    void setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb);

protected:
    static int _interruptsAttached[4];
    static int _interruptsDetached[4];
    static int _processInterrupt[4];

private:
    // Pin assignments
    const uint8_t _signalPin;
    const uint8_t _encoderId;

    // State management
    EncoderState           _state;
    LapState               _lap;
    mutable EncoderContext _encoderContext;

    volatile bool _enabled;

    RPulse _r_pulse;

    // Filtering and timing
    int64_t          _lastPulseTime;
    volatile int64_t _lastFallingEdgeTime;
    volatile int64_t _lastRisingEdgeTime;

    // Interrupt handling
    volatile bool _newPulseAvailable;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    volatile bool _bufferUpdated;

    int32_t _last_pulse;
    bool    _initialized;

    // Maximum number of encoders supported
    static constexpr uint8_t MAX_ENCODERS        = 4;
    static constexpr int8_t  LAPS_OFFSET         = 10;
    static constexpr int64_t DIR_THRESHOLD       = 2;     // For example, if the difference is more than 2 pulses → change direction
    static constexpr int64_t FULL_SCALE          = 4096;  // 0..4095
    static constexpr int64_t HIGH_WRAP_THRESHOLD = 1000;
    static constexpr int64_t LOW_WRAP_THRESHOLD  = -1000;
    static constexpr float   LEAD_SCREW_PITCH_MM = 0.2f;  // Lead screw pitch in mm
    static constexpr size_t  PULSE_BUFFER_SIZE   = 5;     // Pulse width ring buffers

    // static portMUX_TYPE      classMux;
    mutable portMUX_TYPE classMux = portMUX_INITIALIZER_UNLOCKED;

    static MAE3Encoder* _encoderInstances[MAX_ENCODERS];  // Static array to store encoder instances for interrupt handling

    std::array<int64_t, PULSE_BUFFER_SIZE> _width_l_buffer{};
    std::array<int64_t, PULSE_BUFFER_SIZE> _width_h_buffer{};
    size_t                                 _pulseBufferIndex = 0;

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

    void setPeriod(int32_t lapIndex, int64_t period, bool reset_count = false);
    void resetAllPeriods();
};

#endif  // MAE3_ENCODER2_H
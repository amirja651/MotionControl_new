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

#define ENCODER_RESOLUTION 4096  // Encoder 12 bits

struct votePair
{
    int32_t value;
    int32_t count;
};

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

    EncoderState getState() const
    {
        return _state;
    }

    uint32_t getInterruptCount() const;
    uint32_t getHighEdgeCount() const;
    uint32_t getLowEdgeCount() const;
    void     resetInterruptCounters();

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

    // Voting mechanism for encoder readings
    static constexpr size_t                 VOTING_BUFFER_SIZE = 15;    // Number of readings to vote on
    std::array<int32_t, VOTING_BUFFER_SIZE> _votingBuffer{};            // Buffer for voting
    size_t                                  _votingIndex      = 0;      // Current index in voting buffer
    bool                                    _votingBufferFull = false;  // Whether voting buffer is full

    // Interrupt counters
    volatile uint32_t _interruptCount = 0;  // Total interrupt count
    volatile uint32_t _highEdgeCount  = 0;  // Rising edge count
    volatile uint32_t _lowEdgeCount   = 0;  // Falling edge count

    std::function<void(const EncoderState&)> onPulseUpdated;  // NEW: callback support

    void attachInterruptHandler();  // Attaches high-priority GPIO interrupt
    void detachInterruptHandler();

    void IRAM_ATTR processInterrupt();

    int64_t get_median_width_high() const;
    int64_t get_median_width_low() const;

    // Voting mechanism helper methods
    votePair getMostFrequentValue() const;  // Returns the most frequently occurring value in voting buffer

    static void IRAM_ATTR interruptHandler0(void* arg);
    static void IRAM_ATTR interruptHandler1(void* arg);
    static void IRAM_ATTR interruptHandler2(void* arg);
    static void IRAM_ATTR interruptHandler3(void* arg);
};

#endif  // MAE3_ENCODER2_H
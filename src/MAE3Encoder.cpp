#include "MAE3Encoder.h"
#include <algorithm>
#include <bitset>
#include <esp_timer.h>

// For pins above 31 (e.g. GPIO32 to GPIO39), GPIO.in1.data is used
#define READ_FAST(pin) ((pin < 32) ? ((GPIO.in >> pin) & 0x1) : ((GPIO.in1.data >> (pin - 32)) & 0x1))

// Initialize static member
MAE3Encoder* MAE3Encoder::encoderInstances[MAX_ENCODERS] = {nullptr};

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder::interruptHandler0()
{
    if (encoderInstances[0] && encoderInstances[0]->enabled)
    {
        encoderInstances[0]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler1()
{
    if (encoderInstances[1] && encoderInstances[1]->enabled)
    {
        encoderInstances[1]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler2()
{
    if (encoderInstances[2] && encoderInstances[2]->enabled)
    {
        encoderInstances[2]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler3()
{
    if (encoderInstances[3] && encoderInstances[3]->enabled)
    {
        encoderInstances[3]->processInterrupt();
    }
}

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : signalPin(signalPin),
      encoderId(encoderId),
      state{},
      lap{},
      encoderContext{},
      enabled(false),
      lastPulseTime(0),
      lastFallingEdgeTime(0),
      lastRisingEdgeTime(0),
      newPulseAvailable(false),
      bufferUpdated(false),
      width_l_buffer{},
      width_h_buffer{},
      pulseBufferIndex(0),
      last_pulse(0),
      initialized(0)
{
}

MAE3Encoder::~MAE3Encoder()
{
    disable();
    if (encoderId < MAX_ENCODERS)
    {
        encoderInstances[encoderId] = nullptr;
    }
}

bool MAE3Encoder::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        return false;
    }

    // Configure pins
    pinMode(signalPin, INPUT);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Initialize state
    reset();

    // Encoder starts disabled by default
    enabled = false;

    return true;
}

void MAE3Encoder::enable()
{
    if (enabled)
        return;

    enabled = true;
    attachInterruptHandler();
}

void MAE3Encoder::disable()
{
    if (!enabled)
        return;

    enabled = false;
    detachInterruptHandler();
}

void MAE3Encoder::reset()
{
    state.current_pulse = 0;
    state.width_high    = 0;
    state.width_low     = 0;
    state.direction     = Direction::UNKNOWN;

    encoderContext.current_pulse = 0;
    encoderContext.lap_id        = 0;
    encoderContext.lap_period    = 0;
    // encoderContext.average_period   = 0;
    encoderContext.position_degrees = 0;
    encoderContext.position_mm      = 0;
    encoderContext.total_travel_mm  = 0;
    encoderContext.total_travel_um  = 0;
    encoderContext.direction        = "UNK";

    resetAllPeriods();

    last_pulse          = 0;
    lastPulseTime       = 0;
    lastFallingEdgeTime = 0;
    lastRisingEdgeTime  = 0;
    pulseBufferIndex    = 0;

    newPulseAvailable = false;
    bufferUpdated     = false;
    initialized       = false;
}

void MAE3Encoder::attachInterruptHandler()
{
    switch (encoderId)
    {
        case 0:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler3, CHANGE);
            break;
    }
}

void MAE3Encoder::detachInterruptHandler()
{
    detachInterrupt(digitalPinToInterrupt(signalPin));
}

void IRAM_ATTR MAE3Encoder::processInterrupt()
{
    if (__builtin_expect(!enabled, 0))
        return;

    int64_t currentTime = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&mux);

    if (READ_FAST(signalPin))
    {
        // Rising edge
        lastRisingEdgeTime = currentTime;

        if (lastFallingEdgeTime != 0)
        {
            int64_t pulse_width = lastRisingEdgeTime - lastFallingEdgeTime;

            if (pulse_width < 1 || pulse_width > 4097)
            {
                portEXIT_CRITICAL_ISR(&mux);
                return;
            }

            r_pulse.low = pulse_width;
        }
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime = currentTime;

        if (lastRisingEdgeTime != 0)
        {
            int64_t pulse_width = lastFallingEdgeTime - lastRisingEdgeTime;

            if (pulse_width < 1 || pulse_width > 4097)
            {
                portEXIT_CRITICAL_ISR(&mux);
                return;
            }

            r_pulse.high = pulse_width;
        }

        int64_t period = r_pulse.high + r_pulse.low;

        if (period > 4098 || ((r_pulse.low < 50 && r_pulse.high < 50)))
        {
            r_pulse.high = 0;
            r_pulse.low  = 0;
            portEXIT_CRITICAL_ISR(&mux);
            return;
        }

        state.width_high = width_h_buffer[pulseBufferIndex] = r_pulse.high;
        state.width_low = width_l_buffer[pulseBufferIndex] = r_pulse.low;

        pulseBufferIndex = (pulseBufferIndex + 1) % PULSE_BUFFER_SIZE;
        bufferUpdated    = true;
    }

    portEXIT_CRITICAL_ISR(&mux);
}

// New method for processing PWM signal
void MAE3Encoder::processPWM()
{
    bool updated;

    portENTER_CRITICAL(&mux);
    updated = bufferUpdated;
    portEXIT_CRITICAL(&mux);

    if (!enabled || !updated)
        return;

    int64_t width_h = get_median_width_high();
    int64_t width_l = get_median_width_low();
    int64_t period  = width_h + width_l;

    if (period == 0)
        return;

    // Optimized calculation for x_measured
    int32_t x_measured = ((width_h * 4098) / period) - 1;

    // Validate based on documentation
    if (x_measured > FULL_SCALE || x_measured == state.current_pulse)
        return;

    state.current_pulse = (x_measured >= 4095) ? 4095 : x_measured;

    if (!initialized)
    {
        last_pulse  = state.current_pulse;
        initialized = true;

        resetAllPeriods();
        setPeriod(lap.id, period, true);

        return;
    }

    setPeriod(lap.id, period);

    int32_t delta = state.current_pulse - last_pulse;

    if (delta > HIGH_WRAP_THRESHOLD)
    {
        lap.id--;
        state.direction = Direction::CLOCKWISE;
    }
    else if (delta < LOW_WRAP_THRESHOLD)
    {
        lap.id++;
        state.direction = Direction::COUNTER_CLOCKWISE;
    }
    else
    {
        // Small changes → normal direction
        delta                  = state.current_pulse - last_pulse;
        int32_t delta_circular = ((delta + FULL_SCALE / 2) % FULL_SCALE) - FULL_SCALE / 2;

        if (delta_circular > DIR_THRESHOLD)
            state.direction = Direction::CLOCKWISE;
        else if (delta_circular < -DIR_THRESHOLD)
            state.direction = Direction::COUNTER_CLOCKWISE;
    }

    // Update
    last_pulse = state.current_pulse;

    newPulseAvailable = true;
    portENTER_CRITICAL(&mux);
    bufferUpdated = false;
    portEXIT_CRITICAL(&mux);
    lastPulseTime = esp_timer_get_time();

    // NEW: invoke callback if set
    if (onPulseUpdated)
    {
        onPulseUpdated(state);
    }
}

int64_t MAE3Encoder::get_median_width_high() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    portENTER_CRITICAL(&mux);
    // Use memcpy for faster copy inside critical section
    memcpy(temp.data(), width_h_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);
    portEXIT_CRITICAL(&mux);

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

int64_t MAE3Encoder::get_median_width_low() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    portENTER_CRITICAL(&mux);
    // Use memcpy for faster copy inside critical section
    memcpy(temp.data(), width_l_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);
    portEXIT_CRITICAL(&mux);

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

#include "MAE3Encoder.h"

// For pins above 31 (e.g. GPIO32 to GPIO39), GPIO.in1.data is used
#define READ_FAST(pin) ((pin < 32) ? ((GPIO.in >> pin) & 0x1) : ((GPIO.in1.data >> (pin - 32)) & 0x1))

// Initialize static member
MAE3Encoder* MAE3Encoder::_encoderInstances[MAX_ENCODERS] = {nullptr};

portMUX_TYPE MAE3Encoder::classMux = portMUX_INITIALIZER_UNLOCKED;

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder::interruptHandler0()
{
    if (_encoderInstances[0] && _encoderInstances[0]->_enabled)
        _encoderInstances[0]->processInterrupt();
}

void IRAM_ATTR MAE3Encoder::interruptHandler1()
{
    if (_encoderInstances[1] && _encoderInstances[1]->_enabled)
        _encoderInstances[1]->processInterrupt();
}

void IRAM_ATTR MAE3Encoder::interruptHandler2()
{
    if (_encoderInstances[2] && _encoderInstances[2]->_enabled)
        _encoderInstances[2]->processInterrupt();
}

void IRAM_ATTR MAE3Encoder::interruptHandler3()
{
    if (_encoderInstances[3] && _encoderInstances[3]->_enabled)
        _encoderInstances[3]->processInterrupt();
}

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : _signalPin(signalPin),
      _encoderId(encoderId),
      _state{},
      _lap{},
      _encoderContext{},
      _enabled(false),
      _lastPulseTime(0),
      _lastFallingEdgeTime(0),
      _lastRisingEdgeTime(0),
      _newPulseAvailable(false),
      _bufferUpdated(false),
      _width_l_buffer{},
      _width_h_buffer{},
      _pulseBufferIndex(0),
      _last_pulse(0),
      _initialized(0)
{
}

MAE3Encoder::~MAE3Encoder()
{
    disable();
    if (_encoderId < MAX_ENCODERS)
        _encoderInstances[_encoderId] = nullptr;
}

bool MAE3Encoder::begin()
{
    if (_encoderId >= MAX_ENCODERS)
        return false;

    // Configure pins
    pinMode(_signalPin, INPUT);

    // Store instance for interrupt handling
    _encoderInstances[_encoderId] = this;

    // Initialize state
    reset();

    // Encoder starts disabled by default
    _enabled = false;

    return true;
}

void MAE3Encoder::enable()
{
    if (_enabled)
        return;

    _enabled = true;
    attachInterruptHandler();
}

void MAE3Encoder::disable()
{
    if (!_enabled)
        return;

    _enabled = false;
    detachInterruptHandler();
}

void MAE3Encoder::reset()
{
    _state.current_pulse = 0;
    _state.width_high    = 0;
    _state.width_low     = 0;
    _state.direction     = Direction::UNKNOWN;

    _encoderContext.current_pulse = 0;
    _encoderContext.lap_id        = 0;
    _encoderContext.lap_period    = 0;
    // encoderContext.average_period   = 0;
    _encoderContext.position_degrees = 0;
    _encoderContext.position_mm      = 0;
    _encoderContext.total_travel_mm  = 0;
    _encoderContext.total_travel_um  = 0;
    _encoderContext.direction        = "UNK";

    resetAllPeriods();

    _last_pulse          = 0;
    _lastPulseTime       = 0;
    _lastFallingEdgeTime = 0;
    _lastRisingEdgeTime  = 0;
    _pulseBufferIndex    = 0;

    _newPulseAvailable = false;
    _bufferUpdated     = false;
    _initialized       = false;
}

void MAE3Encoder::attachInterruptHandler()
{
    switch (_encoderId)
    {
        case 0:
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler3, CHANGE);
            break;
    }
}

void MAE3Encoder::detachInterruptHandler()
{
    detachInterrupt(digitalPinToInterrupt(_signalPin));
}

void IRAM_ATTR MAE3Encoder::processInterrupt()
{
    if (__builtin_expect(!_enabled, 0))
        return;

    int64_t currentTime = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&classMux);

    if (READ_FAST(_signalPin))
    {
        // Rising edge
        _lastRisingEdgeTime = currentTime;

        if (_lastFallingEdgeTime != 0)
        {
            int64_t pulse_width = _lastRisingEdgeTime - _lastFallingEdgeTime;

            if (pulse_width < 1 || pulse_width > 4097)
            {
                portEXIT_CRITICAL_ISR(&classMux);
                return;
            }

            _r_pulse.low = pulse_width;
        }
    }
    else
    {
        // Falling edge
        _lastFallingEdgeTime = currentTime;

        if (_lastRisingEdgeTime != 0)
        {
            int64_t pulse_width = _lastFallingEdgeTime - _lastRisingEdgeTime;

            if (pulse_width < 1 || pulse_width > 4097)
            {
                portEXIT_CRITICAL_ISR(&classMux);
                return;
            }

            _r_pulse.high = pulse_width;
        }

        int64_t period = _r_pulse.high + _r_pulse.low;

        if (period > 4098 || ((_r_pulse.low < 50 && _r_pulse.high < 50)))
        {
            _r_pulse.high = 0;
            _r_pulse.low  = 0;
            portEXIT_CRITICAL_ISR(&classMux);
            return;
        }

        _state.width_high = _width_h_buffer[_pulseBufferIndex] = _r_pulse.high;
        _state.width_low = _width_l_buffer[_pulseBufferIndex] = _r_pulse.low;

        _pulseBufferIndex = (_pulseBufferIndex + 1) % _PULSE_BUFFER_SIZE;
        _bufferUpdated    = true;
    }

    portEXIT_CRITICAL_ISR(&classMux);
}

// New method for processing PWM signal
void MAE3Encoder::processPWM()
{
    bool updated;

    portENTER_CRITICAL(&classMux);
    updated = _bufferUpdated;
    portEXIT_CRITICAL(&classMux);

    if (!_enabled || !updated)
        return;

    int64_t width_h = get_median_width_high();
    int64_t width_l = get_median_width_low();
    int64_t period  = width_h + width_l;

    if (period == 0)
        return;

    // Optimized calculation for x_measured
    int32_t x_measured = ((width_h * 4098) / period) - 1;

    // Validate based on documentation
    if (x_measured > FULL_SCALE || x_measured == _state.current_pulse)
        return;

    _state.current_pulse = (x_measured >= 4095) ? 4095 : x_measured;

    if (!_initialized)
    {
        _last_pulse  = _state.current_pulse;
        _initialized = true;

        resetAllPeriods();
        setPeriod(_lap.id, period, true);

        return;
    }

    setPeriod(_lap.id, period);

    int32_t delta = _state.current_pulse - _last_pulse;

    if (delta > HIGH_WRAP_THRESHOLD)
    {
        _lap.id--;
        _state.direction = Direction::CLOCKWISE;
    }
    else if (delta < LOW_WRAP_THRESHOLD)
    {
        _lap.id++;
        _state.direction = Direction::COUNTER_CLOCKWISE;
    }
    else
    {
        // Small changes → normal direction
        delta                  = _state.current_pulse - _last_pulse;
        int32_t delta_circular = ((delta + FULL_SCALE / 2) % FULL_SCALE) - FULL_SCALE / 2;

        if (delta_circular > DIR_THRESHOLD)
            _state.direction = Direction::CLOCKWISE;
        else if (delta_circular < -DIR_THRESHOLD)
            _state.direction = Direction::COUNTER_CLOCKWISE;
    }

    // Update
    _last_pulse = _state.current_pulse;

    _newPulseAvailable = true;
    portENTER_CRITICAL(&classMux);
    _bufferUpdated = false;
    portEXIT_CRITICAL(&classMux);
    _lastPulseTime = esp_timer_get_time();

    // NEW: invoke callback if set
    if (onPulseUpdated)
    {
        onPulseUpdated(_state);
    }
}

int64_t MAE3Encoder::get_median_width_high() const
{
    std::array<int64_t, _PULSE_BUFFER_SIZE> temp;

    portENTER_CRITICAL(&classMux);
    // Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_h_buffer.data(), sizeof(int64_t) * _PULSE_BUFFER_SIZE);
    portEXIT_CRITICAL(&classMux);

    std::sort(temp.begin(), temp.end());
    return temp[_PULSE_BUFFER_SIZE / 2];
}

int64_t MAE3Encoder::get_median_width_low() const
{
    std::array<int64_t, _PULSE_BUFFER_SIZE> temp;

    portENTER_CRITICAL(&classMux);
    // Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_l_buffer.data(), sizeof(int64_t) * _PULSE_BUFFER_SIZE);
    portEXIT_CRITICAL(&classMux);

    std::sort(temp.begin(), temp.end());
    return temp[_PULSE_BUFFER_SIZE / 2];
}

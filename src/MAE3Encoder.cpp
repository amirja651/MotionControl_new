#include "MAE3Encoder.h"

// Initialize static member
MAE3Encoder* MAE3Encoder::_encoderInstances[MAX_ENCODERS] = {nullptr};

int MAE3Encoder::_interruptsAttached[NUM_ENCODERS] = {0};
int MAE3Encoder::_interruptsDetached[NUM_ENCODERS] = {0};
// Initialize static member
portMUX_TYPE MAE3Encoder::classMux = portMUX_INITIALIZER_UNLOCKED;

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : _signalPin(signalPin),
      _encoderId(encoderId),
      _state{},
      _lap{},
      _encoderContext{},
      _enabled(false),
      _r_pulse{0, 0},
      _lastPulseTime(0),
      _lastFallingEdgeTime(0),
      _lastRisingEdgeTime(0),
      _newPulseAvailable(false),
      _bufferUpdated(false),
      _last_pulse(0),
      _initialized(0),
      _width_l_buffer{},
      _width_h_buffer{},
      _pulseBufferIndex(0)
{
#if DEBUG_ENCODER
    Serial.print(F("Encoder constructor: "));
    Serial.println(_encoderId + 1);
#endif
}

MAE3Encoder::~MAE3Encoder()
{
    disable();

#if DEBUG_ENCODER
    Serial.print(F("Encoder destructor: "));
    Serial.println(_encoderId + 1);
#endif

    if (_encoderId < MAX_ENCODERS)
    {
#if DEBUG_ENCODER
        Serial.print(F(" - encoderInstances set to nullptr"));
        Serial.println(_encoderId);
#endif
        _encoderInstances[_encoderId] = nullptr;
    }
}

bool MAE3Encoder::begin()
{
    if (_encoderId >= MAX_ENCODERS)
        return false;

    // Initialize state
    reset();

    // Configure pins
    pinMode(_signalPin, INPUT);

    // Encoder starts disabled by default
    _enabled = false;

    // Store instance for interrupt handling
    _encoderInstances[_encoderId] = this;

#if DEBUG_ENCODER
    Serial.print(F("Encoder "));
    Serial.print(_encoderId + 1);
    Serial.println(F(" initialized"));
    Serial.println();
#endif

    return true;
}

void MAE3Encoder::enable()
{
    if (_enabled)
        return;

    attachInterruptHandler();
    _enabled = true;
}
void MAE3Encoder::disable()
{
    if (!_enabled)
        return;

    detachInterruptHandler();
    _enabled = false;
}

bool MAE3Encoder::isEnabled() const
{
    return _enabled;
}
bool MAE3Encoder::isDisabled() const
{
    return !_enabled;
}

void MAE3Encoder::reset()
{
    // Reset state
    _state.current_pulse = 0;
    _state.width_high    = 0;
    _state.width_low     = 0;
    _state.direction     = Direction::UNKNOWN;

    // Reset LapState
    resetAllPeriods();

    _encoderContext.current_pulse    = 0;
    _encoderContext.lap_id           = 0;
    _encoderContext.lap_period       = 0;
    _encoderContext.position_degrees = 0;
    _encoderContext.position_mm      = 0;
    _encoderContext.total_travel_mm  = 0;
    _encoderContext.total_travel_um  = 0;
    _encoderContext.direction        = "UNK";

    // Reset RPulse
    _r_pulse.low  = 0;
    _r_pulse.high = 0;

    // Reset enabled
    _enabled = false;

    // Filtering and timing
    _lastPulseTime       = 0;
    _lastFallingEdgeTime = 0;
    _lastRisingEdgeTime  = 0;

    // Interrupt handling
    _newPulseAvailable = false;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    _bufferUpdated = false;

    // Reset last pulse
    _last_pulse  = 0;
    _initialized = false;

#if DEBUG_ENCODER
    Serial.print(F("Encoder reset: "));
    Serial.println(_encoderId + 1);
#endif
}

//  method for processing PWM signal *************************amir
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

EncoderContext& MAE3Encoder::getEncoderContext() const
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

int32_t MAE3Encoder::umToPulses(float um)
{
    float mm            = um / 1000.0f;
    float pulses_per_mm = FULL_SCALE / LEAD_SCREW_PITCH_MM;
    return static_cast<int32_t>(mm * pulses_per_mm);
}
int32_t MAE3Encoder::degreesToPulses(float degrees)
{
    float pulses_per_degree = FULL_SCALE / 360.0f;
    return static_cast<int32_t>(degrees * pulses_per_degree);
}
float MAE3Encoder::pulsesToUm(float pulses)
{
    float mm = pulses * (LEAD_SCREW_PITCH_MM / FULL_SCALE);
    return mm * 1000.0f;
}

bool MAE3Encoder::isStopped(int64_t threshold_us) const
{
    return (esp_timer_get_time() - _lastPulseTime) > threshold_us;
}
void MAE3Encoder::setOnPulseUpdatedCallback(std::function<void(const EncoderState&)> cb)
{
    onPulseUpdated = cb;
}

void MAE3Encoder::attachInterruptHandler()
{
    switch (_encoderId)
    {
        case 0:
#if NUM_ENCODERS > 0
    #if DEBUG_ENCODER
            _interruptsAttached[_encoderId]++;
    #endif
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler0, CHANGE);
#endif
            break;
#if NUM_ENCODERS > 1
        case 1:
    #if DEBUG_ENCODER
            _interruptsAttached[_encoderId]++;
    #endif
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler1, CHANGE);

            break;
#endif
#if NUM_ENCODERS > 2
        case 2:
    #if DEBUG_ENCODER
            _interruptsAttached[_encoderId]++;
    #endif
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler2, CHANGE);

            break;
#endif
#if NUM_ENCODERS > 3
        case 3:
    #if DEBUG_ENCODER
            _interruptsAttached[_encoderId]++;
    #endif
            attachInterrupt(digitalPinToInterrupt(_signalPin), interruptHandler3, CHANGE);

            break;
#endif
    }
}
void MAE3Encoder::detachInterruptHandler()
{
#if DEBUG_ENCODER
    _interruptsDetached[_encoderId]++;
#endif
    detachInterrupt(digitalPinToInterrupt(_signalPin));
}

// method for processing interrupt *************************amir
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

        _pulseBufferIndex = (_pulseBufferIndex + 1) % PULSE_BUFFER_SIZE;
        _bufferUpdated    = true;
    }

    portEXIT_CRITICAL_ISR(&classMux);
}

// Get median width of high and low pulses
int64_t MAE3Encoder::get_median_width_high() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    portENTER_CRITICAL(&classMux);
    // Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_h_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);
    portEXIT_CRITICAL(&classMux);

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}
int64_t MAE3Encoder::get_median_width_low() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    portENTER_CRITICAL(&classMux);
    // Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_l_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);
    portEXIT_CRITICAL(&classMux);

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

// Individual interrupt handlers for each encoder
#if NUM_ENCODERS > 0
void IRAM_ATTR MAE3Encoder::interruptHandler0()
{
    #if DEBUG_ENCODER
    Serial.print(F("Encoder interruptHandler: "));
    #endif
    if (_encoderInstances[0] && _encoderInstances[0]->_enabled)
    {
    #if DEBUG_ENCODER
        Serial.print(_encoderInstances[0]->_encoderId + 1);
    #endif
        _encoderInstances[0]->processInterrupt();
    }
    #if DEBUG_ENCODER
    Serial.println();
    #endif
}
#endif
#if NUM_ENCODERS > 1
void IRAM_ATTR MAE3Encoder::interruptHandler1()
{
    #if DEBUG_ENCODER
    Serial.print(F("Encoder interruptHandler: "));
    #endif
    if (_encoderInstances[1] && _encoderInstances[1]->_enabled)
    {
    #if DEBUG_ENCODER
        Serial.print(_encoderInstances[1]->_encoderId + 1);
    #endif
        _encoderInstances[1]->processInterrupt();
    }
    #if DEBUG_ENCODER
    Serial.println();
    #endif
}
#endif
#if NUM_ENCODERS > 2
void IRAM_ATTR MAE3Encoder::interruptHandler2()
{
    #if DEBUG_ENCODER
    Serial.print(F("Encoder interruptHandler: "));
    #endif
    if (_encoderInstances[2] && _encoderInstances[2]->_enabled)
    {
    #if DEBUG_ENCODER
        Serial.print(_encoderInstances[2]->_encoderId + 1);
    #endif
        _encoderInstances[2]->processInterrupt();
    }
    #if DEBUG_ENCODER
    Serial.println();
    #endif
}
#endif
#if NUM_ENCODERS > 3
void IRAM_ATTR MAE3Encoder::interruptHandler3()
{
    #if DEBUG_ENCODER
    Serial.print(F("Encoder interruptHandler: "));
    #endif
    if (_encoderInstances[3] && _encoderInstances[3]->_enabled)
    {
    #if DEBUG_ENCODER
        Serial.print(_encoderInstances[3]->_encoderId + 1);
    #endif
        _encoderInstances[3]->processInterrupt();
    }
    #if DEBUG_ENCODER
    Serial.println();
    #endif
}
#endif
void MAE3Encoder::setPeriod(int32_t lapIndex, int64_t period, bool reset_count)
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
void MAE3Encoder::resetAllPeriods()
{
    _lap.id = 0;
    memset(_lap.period, 0, sizeof(_lap.period));
    memset(_lap.period_sum, 0, sizeof(_lap.period_sum));
    memset(_lap.period_count, 0, sizeof(_lap.period_count));
}

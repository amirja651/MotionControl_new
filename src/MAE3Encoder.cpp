#include "MAE3Encoder.h"

// Initialize static member
MAE3Encoder* MAE3Encoder::_encoderInstances[4] = {nullptr};

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : _signalPin(signalPin), _encoderId(encoderId), _state{}, _lap{}, _encoderContext{}, _vResult{}, _enabled(false), _r_pulse{0, 0}, _lastPulseTime(0), _lastFallingEdgeTime(0), _lastRisingEdgeTime(0), _newPulseAvailable(false), _bufferUpdated(false), _newData(false), _last_pulse(0), _initialized(0), _width_l_buffer{}, _width_h_buffer{}, _pulseBufferIndex(0)
{
}

MAE3Encoder::~MAE3Encoder()
{
    disable();
    _encoderInstances[_encoderId] = nullptr;
}

bool MAE3Encoder::begin()
{
    // Initialize state
    reset();

    // Configure pins
    pinMode(_signalPin, INPUT);

    // Encoder starts disabled by default
    _enabled = false;

    // Store instance for interrupt handling
    _encoderInstances[_encoderId] = this;

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

    // Reset validatePWMResult
    resetValidatePwmResult();

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
    _newData       = false;

    // Reset last pulse
    _last_pulse  = 0;
    _initialized = false;
}

//  method for processing PWM signal *************************amir
void MAE3Encoder::processPWM()
{
    if (_newData)
    {
        validateResult vResult = validatePWMValues(_r_pulse.high, _r_pulse.low, _signalPin);

        if (!vResult.overall)
        {
            _r_pulse.high = 0;
            _r_pulse.low  = 0;
            _newData      = false;
            return;
        }

        _state.width_high = _width_h_buffer[_pulseBufferIndex] = _r_pulse.high;
        _state.width_low = _width_l_buffer[_pulseBufferIndex] = _r_pulse.low;

        _pulseBufferIndex = (_pulseBufferIndex + 1) % PULSE_BUFFER_SIZE;
        _bufferUpdated    = true;
        _newData          = false;
    }

    if (!_bufferUpdated)  // if (!_enabled || !updated)
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

    _bufferUpdated = false;

    _lastPulseTime = esp_timer_get_time();

    // NEW: invoke callback if set
    if (onPulseUpdated)
    {
        onPulseUpdated(_state);
    }
}

EncoderContext& MAE3Encoder::getEncoderContext() const
{
    _encoderContext.current_pulse = _state.current_pulse;
    _encoderContext.direction     = _state.direction == Direction::UNKNOWN ? "   " : _state.direction == Direction::CLOCKWISE ? " CW" : "CCW";
    _encoderContext.lap_id        = _lap.id;
    _encoderContext.lap_period    = _lap.period[_lap.id + LAPS_OFFSET];

    _encoderContext.position_degrees = _state.current_pulse * (360.0f / FULL_SCALE);
    _encoderContext.position_mm      = _encoderContext.current_pulse * (LEAD_SCREW_PITCH_MM / FULL_SCALE);
    _encoderContext.total_travel_mm  = (_lap.id * LEAD_SCREW_PITCH_MM) + _encoderContext.position_mm;
    _encoderContext.total_travel_um  = _encoderContext.total_travel_mm * 1000.0f;

    return _encoderContext;
}

validateResult& MAE3Encoder::getValidatePWMResult() const
{
    return _vResult;
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

// method for processing interrupt *************************amir
void IRAM_ATTR MAE3Encoder::processInterrupt()
{
    // if (_bufferUpdated)
    //  return;

    int64_t currentTime = esp_timer_get_time();
    if (digitalRead(_signalPin) == HIGH)
    {
        // Rising edge
        _lastRisingEdgeTime = currentTime;
        if (_lastFallingEdgeTime != 0)
            _r_pulse.low = _lastRisingEdgeTime - _lastFallingEdgeTime;
    }
    else
    {
        // Falling edge
        _lastFallingEdgeTime = currentTime;
        if (_lastRisingEdgeTime != 0)
        {
            _r_pulse.high = _lastFallingEdgeTime - _lastRisingEdgeTime;
            _newData      = true;
        }
    }
}

// Get median width of high and low pulses
int64_t MAE3Encoder::get_median_width_high() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    //  Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_h_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}
int64_t MAE3Encoder::get_median_width_low() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    //  Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_l_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder::interruptHandler0()
{
    _encoderInstances[0]->processInterrupt();
}
void IRAM_ATTR MAE3Encoder::interruptHandler1()
{
    _encoderInstances[1]->processInterrupt();
}
void IRAM_ATTR MAE3Encoder::interruptHandler2()
{
    _encoderInstances[2]->processInterrupt();
}
void IRAM_ATTR MAE3Encoder::interruptHandler3()
{
    _encoderInstances[3]->processInterrupt();
}
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

// Diagnostic function to check encoder signal quality
void MAE3Encoder::diagnoseEncoderSignals()
{
    // Check for signal activity
    printf("\nSignal Activity Test (5 seconds):\n");

    unsigned long startTime   = millis();
    unsigned long transitions = 0;
    bool          lastState   = digitalRead(_signalPin);

    while (millis() - startTime < 5000)
    {
        bool currentState = digitalRead(_signalPin);

        if (currentState != lastState)
        {
            transitions++;
            lastState = currentState;
        }

        delayMicroseconds(100);
    }

    Serial.println();
    printf("┌───────────────────────────────────────────────────────────────────────┐\n");
    printf("│                    ENCODER SIGNAL DIAGNOSTICS                         │\n");
    printf("├───────────────────────────────────────────────────────────────────────┤\n");
    printf("│ %-25s │ %-15s │ %-23s │\n", "Parameter", "Value", "Status");
    printf("├───────────────────────────────────────────────────────────────────────┤\n");

    // Interrupt Status
    bool interruptAttached = (digitalPinToInterrupt(_signalPin) != NOT_AN_INTERRUPT);
    printf("│ %-25s │ %-15s │ %-24s │\n", "Interrupt Attached", interruptAttached ? "YES" : "NO", interruptAttached ? "✅ OK" : "❌ FAIL");

    // Pin State
    bool pinState = digitalRead(_signalPin);
    printf("│ %-25s │ %-15s │ %-23s │\n", "Pin State", pinState ? "HIGH" : "LOW", "");

    // Transition Count
    float frequency = (float)transitions / 10.0;
    printf("│ %-25s │ %-15lu │ %-23s │\n", "Transitions", transitions, "");

    // Frequency
    printf("│ %-25s │ %-15.1f │ %-23s │\n", "Measured Frequency", frequency, "");

    // Expected Frequency Range
    printf("│ %-25s │ %-15s │ %-23s │\n", "Expected Frequency", "220-268 Hz", "");

    // Frequency Status
    bool freqOK = (frequency >= 220.0 && frequency <= 268.0);
    printf("│ %-25s │ %-15s │ %-24s │\n", "Frequency Status", freqOK ? "IN RANGE" : "OUT OF RANGE", freqOK ? "✅ PASS" : "❌ FAIL");

    // Low Activity Warning
    if (transitions < 100)
    {
        printf("│ %-25s │ %-15s │ %-28s │\n", "Activity Warning", "LOW", "⚠️  Check Wiring!");
    }

    printf("└───────────────────────────────────────────────────────────────────────┘\n\n");
}

validateResult MAE3Encoder::validatePWMValues(int64_t highPulse, int64_t lowPulse, uint8_t pinNumber)
{
    resetValidatePwmResult();

    _vResult.totalPeriod = highPulse + lowPulse;

    Serial.println();
    printf("┌───────────────────────────────────────────────────────────────────────┐\n");
    printf("│                         Validation for Pin %-4d                       │\n", _signalPin);
    printf("├───────────────────────────────────────────────────────────────────────┤\n");
    printf("│ %-15s │ %-15s │ %-15s │ %-15s │\n", "Parameter", "High Pulse", "Low Pulse", "Total Period");
    printf("├───────────────────────────────────────────────────────────────────────┤\n");
    printf("│ %-15s │ %-15lld │ %-15lld │ %-15lld │\n", "Duration (us)", highPulse, lowPulse, _vResult.totalPeriod);
    printf("└───────────────────────────────────────────────────────────────────────┘\n");

    // Check if data was bypassed (values are 0)
    if (highPulse == 0 && lowPulse == 0)
    {
        printf("Data was bypassed - invalid period detected\n");
        return _vResult;
    }

    // Check each condition separately
    _vResult.highOK   = (highPulse >= 1 && highPulse <= 4302);  // 4097
    _vResult.lowOK    = (lowPulse >= 1 && lowPulse <= 4302);    // 4097
    _vResult.totalOK  = ((_vResult.totalPeriod) > 0);
    _vResult.periodOK = ((_vResult.totalPeriod) >= 3731 && (_vResult.totalPeriod) <= 4545);
    _vResult.overall  = _vResult.highOK && _vResult.lowOK && _vResult.totalOK && _vResult.periodOK;

    printf("┌───────────────────────────────────────────────────────────────────────┐\n");
    printf("│                   Validation Results for Id #%-4d                     │\n", _encoderId + 1);
    printf("├───────────────────────────────────────────────────────────────────────┤\n");
    printf("│ %-25s │ %-10s │ %-28s │\n", "Test", "Status", "Range");
    printf("├───────────────────────────────────────────────────────────────────────┤\n");
    printf("│ %-25s │ %-10s │ %-28s │\n", "High pulse validation", _vResult.highOK ? "PASS" : "FAIL", "1-4097 us");
    printf("│ %-25s │ %-10s │ %-28s │\n", "Low pulse validation", _vResult.lowOK ? "PASS" : "FAIL", "1-4097 us");
    printf("│ %-25s │ %-10s │ %-28s │\n", "Total period validation", _vResult.totalOK ? "PASS" : "FAIL", "> 0 us");
    printf("│ %-25s │ %-10s │ %-28s │\n", "Period range validation", _vResult.periodOK ? "PASS" : "FAIL", "3731-4545 us");
    printf("│ %-25s │ %-10s │ %-29s │\n", "Overall validation", _vResult.overall ? "PASS" : "FAIL", "✅");
    printf("└───────────────────────────────────────────────────────────────────────┘\n");

    return _vResult;
}

void MAE3Encoder::resetValidatePwmResult()
{
    _vResult.totalPeriod = 0;
    _vResult.position    = 0;
    _vResult.degrees     = 0;
    _vResult.delta       = 0;
    _vResult.highOK      = false;
    _vResult.lowOK       = false;
    _vResult.totalOK     = false;
    _vResult.periodOK    = false;
    _vResult.overall     = false;
}
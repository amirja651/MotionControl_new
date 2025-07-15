#include "MAE3Encoder.h"
#include <driver/gpio.h>

#define ENCODER_DEBUG true

// Initialize static member
MAE3Encoder* MAE3Encoder::_encoderInstances[4] = {nullptr};

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : _signalPin(signalPin),
      _encoderId(encoderId),
      _state{},
      _r_pulse{0, 0, 0, 0},
      _lastPulseTime(0),
      _lastFallingEdgeTime(0),
      _lastRisingEdgeTime(0),
      _currentRisingEdge(0),
      _lowStart(0),
      _lowEnd(0),
      _enabled(false),
      _dataReady(false),
      _initialized(false),
      _last_pulse(0),
      _width_l_buffer{},
      _width_h_buffer{},
      _pulseBufferIndex(0)
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
    _state.width_high       = 0;
    _state.width_low        = 0;
    _state.width_interval   = 0;
    _state.lap_id           = 0;
    _state.position_pulse   = 0;
    _state.position_mm      = 0;
    _state.position_degrees = 0;
    _state.total_mm         = 0;
    _state.direction        = Direction::UNKNOWN;

    _r_pulse.low               = 0;
    _r_pulse.high              = 0;
    _r_pulse.pulse_Interval    = 0;
    _r_pulse.duration_us_count = 0;

    // Reset enabled
    _enabled = false;

    // Filtering and timing
    _lastPulseTime       = 0;
    _lastFallingEdgeTime = 0;
    _lastRisingEdgeTime  = 0;

    // Flag to indicate buffer was updated (set in processInterrupt, cleared after processPWM)
    _dataReady = false;

    // Reset last pulse
    _last_pulse  = 0;
    _initialized = false;

    // Reset voting mechanism
    _votingBuffer.fill(0);
    _votingIndex      = 0;
    _votingBufferFull = false;
}

//  method for processing PWM signal *** amir
void MAE3Encoder::processPWM(bool print)
{
    if (!_enabled || !_dataReady)
        return;

    noInterrupts();
    int64_t interval = _r_pulse.pulse_Interval;
    int64_t low      = _r_pulse.low;
    int64_t high     = _r_pulse.high;
    _dataReady       = false;

    // Check each condition separately
    bool highOK   = (high >= 1 && high <= 4302);  // 4097
    bool lowOK    = (low >= 1 && low <= 4302);    // 4097
    bool totalOK  = (interval > 0);
    bool periodOK = ((interval) >= 3731 && (interval) <= 4545);
    bool overall  = highOK && lowOK && totalOK && periodOK;

    if (!overall)
    {
        return;
        interrupts();
    }

    // Calculate current encoder reading
    int32_t x_measured = ((high * 4098) / interval) - 1;

    // Validate the reading
    int32_t valid_reading = x_measured;

    interrupts();

    // If reading is invalid, return without updating
    if (valid_reading == -1)
    {
        return;
    }

    // Add reading to voting buffer
    _votingBuffer[_votingIndex] = valid_reading;
    _votingIndex                = (_votingIndex + 1) % VOTING_BUFFER_SIZE;

    // Mark buffer as full after first complete cycle
    if (_votingIndex == 0)
    {
        _votingBufferFull = true;
    }

    // Only process if we have enough readings (buffer is full)
    if (!_votingBufferFull)
    {
        return;
    }

    // Get the most frequent value from voting buffer
    int32_t voted_reading = getMostFrequentValue();

    // Debug output if requested
    if (0)
    {
        Serial.printf("[Encoder %d] Voting Buffer: [", _encoderId + 1);
        for (size_t i = 0; i < VOTING_BUFFER_SIZE; ++i)
        {
            Serial.printf("%d", _votingBuffer[i]);
            if (i < VOTING_BUFFER_SIZE - 1)
                Serial.printf(", ");
        }
        Serial.printf("] -> Voted: %d\n", voted_reading);
    }

    // Use the voted reading as the final position
    if (voted_reading <= 4094)
        _state.position_pulse = voted_reading;
    else if (voted_reading == 4096 || voted_reading == 4097)
        _state.position_pulse = 4095;
    else
    {
        return;
    }

    _state.position_degrees = _state.position_pulse * (360.0f / 4096.0f);
    _state.width_high       = high;
    _state.width_low        = low;
    _state.width_interval   = interval;

    if (!_initialized)
    {
        _last_pulse  = _state.position_pulse;
        _initialized = true;
        return;
    }

    int32_t delta = _state.position_pulse - _last_pulse;

    if (delta > 1000)
    {
        _state.lap_id--;
        _state.direction = Direction::CLOCKWISE;
    }
    else if (delta < -1000)
    {
        _state.lap_id++;
        _state.direction = Direction::COUNTER_CLOCKWISE;
    }
    else
    {
        int32_t delta_circular = ((delta + 4096 / 2) % 4096) - 4096 / 2;

        if (delta_circular > 2)
            _state.direction = Direction::CLOCKWISE;
        else if (delta_circular < -2)
            _state.direction = Direction::COUNTER_CLOCKWISE;
    }

    _last_pulse    = _state.position_pulse;
    _lastPulseTime = esp_timer_get_time();

    if (onPulseUpdated)
    {
        onPulseUpdated(_state);
    }
}

int32_t MAE3Encoder::umToPulses(float um)
{
    float mm            = um / 1000.0f;
    float pulses_per_mm = 4096 / LEAD_SCREW_PITCH_MM;
    return static_cast<int32_t>(mm * pulses_per_mm);
}
int32_t MAE3Encoder::degreesToPulses(float degrees)
{
    float pulses_per_degree = 4096 / 360.0f;
    return static_cast<int32_t>(degrees * pulses_per_degree);
}
float MAE3Encoder::pulsesToUm(float pulses)
{
    float mm = pulses * (LEAD_SCREW_PITCH_MM / 4096);
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
    // Configure GPIO interrupt with high priority
    gpio_config_t io_conf = {};
    io_conf.intr_type     = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask  = (1ULL << _signalPin);
    io_conf.mode          = GPIO_MODE_INPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Install GPIO ISR service with high priority
    // ESP_INTR_FLAG_LEVEL1 = High priority interrupt level
    // This ensures encoder interrupts are handled with minimal latency
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM);

    // Add ISR handler for this specific pin with proper casting
    gpio_isr_handler_add((gpio_num_t)_signalPin,
                         (gpio_isr_t)(_encoderId == 0   ? interruptHandler0
                                      : _encoderId == 1 ? interruptHandler1
                                      : _encoderId == 2 ? interruptHandler2
                                                        : interruptHandler3),
                         (void*)this);
}
void MAE3Encoder::detachInterruptHandler()
{
    // Remove ISR handler for this specific pin
    gpio_isr_handler_remove((gpio_num_t)_signalPin);
}

// method for processing interrupt *** amir
void IRAM_ATTR MAE3Encoder::processInterrupt()
{
    if (!_enabled)
        return;

    int     level       = digitalRead(_signalPin);
    int64_t currentTime = esp_timer_get_time();  // Current time in microseconds
    _r_pulse.duration_us_count++;

    if (level == LOW)
    {
        // Falling edge
        _lowStart = currentTime;
    }
    else
    {
        // Rising edge
        _lowEnd      = currentTime;
        _r_pulse.low = _lowEnd - _lowStart;

        _lastRisingEdgeTime = _currentRisingEdge;
        _currentRisingEdge  = currentTime;

        if (_lastRisingEdgeTime != 0)
        {
            _r_pulse.pulse_Interval = _currentRisingEdge - _lastRisingEdgeTime;
            _r_pulse.high           = _r_pulse.pulse_Interval - _r_pulse.low;
            _dataReady              = true;
        }
    }
}

// Get median width of high and low pulses
int64_t MAE3Encoder::get_median_width_high() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    //  Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_h_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);

    int countNonZero = std::count_if(temp.begin(), temp.end(), [](int64_t v) { return v > 0; });
    if (countNonZero < 3)
        return _state.width_high;  // Not enough valid data yet and return the last valid value

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

int64_t MAE3Encoder::get_median_width_low() const
{
    std::array<int64_t, PULSE_BUFFER_SIZE> temp;

    //  Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_l_buffer.data(), sizeof(int64_t) * PULSE_BUFFER_SIZE);

    int countNonZero = std::count_if(temp.begin(), temp.end(), [](int64_t v) { return v > 0; });
    if (countNonZero < 3)
        return _state.width_low;  // Not enough valid data yet and return the last valid value

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

// Voting mechanism implementation
int32_t MAE3Encoder::getMostFrequentValue() const
{
    // Create a map to count occurrences of each value
    std::unordered_map<int32_t, int> frequency_map;

    // Count occurrences of each value in the voting buffer
    for (size_t i = 0; i < VOTING_BUFFER_SIZE; ++i)
    {
        frequency_map[_votingBuffer[i]]++;
    }

    // Find the value with the highest frequency
    int32_t most_frequent_value = _votingBuffer[0];  // Default to first value
    int     max_frequency       = 0;

    for (const auto& pair : frequency_map)
    {
        if (pair.second > max_frequency)
        {
            max_frequency       = pair.second;
            most_frequent_value = pair.first;
        }
    }

    return most_frequent_value;
}

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder::interruptHandler0(void* arg)
{
    MAE3Encoder* encoder = static_cast<MAE3Encoder*>(arg);
    if (encoder)
        encoder->processInterrupt();
}
void IRAM_ATTR MAE3Encoder::interruptHandler1(void* arg)
{
    MAE3Encoder* encoder = static_cast<MAE3Encoder*>(arg);
    if (encoder)
        encoder->processInterrupt();
}
void IRAM_ATTR MAE3Encoder::interruptHandler2(void* arg)
{
    MAE3Encoder* encoder = static_cast<MAE3Encoder*>(arg);
    if (encoder)
        encoder->processInterrupt();
}
void IRAM_ATTR MAE3Encoder::interruptHandler3(void* arg)
{
    MAE3Encoder* encoder = static_cast<MAE3Encoder*>(arg);
    if (encoder)
        encoder->processInterrupt();
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

float MAE3Encoder::calculateMirrorAngle(int currentPixel, int referencePixel)
{
    // Constants
    const float pixelSize_mm = 0.0052;  // Size of each pixel in millimeters (5.2 micrometers)
    const float distance_mm  = 195.0;   // Distance from mirror to camera in millimeters

    // Pixel difference
    int deltaPixel = currentPixel - referencePixel;

    // Convert pixel difference to millimeters
    float displacement_mm = deltaPixel * pixelSize_mm;

    // Calculate angle in degrees
    float angleRad       = atan(displacement_mm / distance_mm);  // Angle of deviation of the beam (2θ)
    float mirrorAngleDeg = (angleRad * 180.0 / PI) / 2.0;        // Divide by 2 → mirror rotation angle

    return mirrorAngleDeg;
}

float MAE3Encoder::calculateDistanceToMirror(int currentPixel, int referencePixel, float mirrorAngleDeg)
{
    const float pixelSize_mm = 0.0052;  // Size of each pixel (millimeters)

    int   deltaPixel      = currentPixel - referencePixel;
    float displacement_mm = deltaPixel * pixelSize_mm;

    // Beam angle = 2 * mirror angle
    float beamAngleRad = 2.0 * mirrorAngleDeg * PI / 180.0;

    // Prevent division by zero or very small tangent
    if (abs(tan(beamAngleRad)) < 1e-6)
        return -1;  // Error or infinite value

    // Calculate distance L in millimeters
    float L = displacement_mm / tan(beamAngleRad);
    return L;  // in millimeters
}

float MAE3Encoder::calculateEncoderAngle(int currentCount, int previousCount, int encoderResolution, float gearRatio)
{
    // Encoder difference between two moments
    int deltaCount = currentCount - previousCount;

    // Calculate motor rotation angle (degrees)
    float motorAngle = (deltaCount * 360.0) / encoderResolution;

    // Apply gear ratio (optional, if exists)
    float mirrorAngle = motorAngle * gearRatio;

    return mirrorAngle;  // in degrees
}

float MAE3Encoder::pixelToMirrorAngle(int deltaPixel)
{
    float dx_mm          = deltaPixel * PIXEL_SIZE_MM;
    float beamAngleRad   = atan(dx_mm / L_MM);
    float mirrorAngleDeg = (beamAngleRad * 180.0 / PI) / 2.0;
    return mirrorAngleDeg;
}

int MAE3Encoder::mirrorAngleToPulses(float mirrorAngleDeg)
{
    float motorAngleDeg = mirrorAngleDeg / GEAR_RATIO;
    float pulses        = (motorAngleDeg / 360.0f) * TOTAL_PULSES_PER_REV;
    return (int)round(pulses);
}

int MAE3Encoder::pixelToPulses(int deltaPixel)
{
    float angle = pixelToMirrorAngle(deltaPixel);
    return mirrorAngleToPulses(angle);
}

int MAE3Encoder::mirrorAngleToPulses(float mirrorAngleDeg, int microstep, int stepsPerRev, float gearRatio)
{
    float motorAngleDeg = mirrorAngleDeg / gearRatio;
    float pulsesPerRev  = stepsPerRev * microstep;
    float pulses        = (motorAngleDeg / 360.0f) * pulsesPerRev;
    return (int)round(pulses);
}

int MAE3Encoder::encoderToPulses(int encoderValue, int microstep, int stepsPerRev, int encoderResolution)
{
    float pulsesPerRev = stepsPerRev * microstep;
    float motorAngle   = ((float)encoderValue / (float)encoderResolution) * 360.0f;
    float pulses       = (motorAngle / 360.0f) * pulsesPerRev;
    return (int)round(pulses);
}

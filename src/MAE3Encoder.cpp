#include "MAE3Encoder.h"
#include "esp32/clk.h"
#include <driver/gpio.h>
#include <esp_task_wdt.h>

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
      _pulseBufferIndex(0),
      _storageCompleteFlag(false),
      _onComplete(nullptr)
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

    // Reset interrupt counters
    _interruptCount = 0;
    _highEdgeCount  = 0;
    _lowEdgeCount   = 0;
}

//  method for processing PWM signal *** amir
void MAE3Encoder::processPWM(bool print)
{
    if (!_enabled || !_dataReady)
        return;

    noInterrupts();
    _storageCompleteFlag   = true;
    votePair voted_reading = getMostFrequentValue();
    _dataReady             = false;
    interrupts();

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
        Serial.printf("] -> Voted: %d (count: %d)\n", voted_reading.value, voted_reading.count);
    }

    // Use the voted reading as the final position
    if (voted_reading.value <= 4094)
        _state.position_pulse = voted_reading.value;
    else if (voted_reading.value == 4096 || voted_reading.value == 4097)
        _state.position_pulse = 4095;
    else
    {
        return;
    }

    _state.position_degrees = _state.position_pulse * (360.0f / 4096);

    if (!_initialized)
    {
        _last_pulse  = _state.position_pulse;
        _initialized = true;
        return;
    }

    uint32_t delta = _state.position_pulse - _last_pulse;

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
        uint32_t delta_circular = ((delta + 4096 / 2) % 4096) - 4096 / 2;

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

uint32_t MAE3Encoder::umToPulses(float um)
{
    float mm            = um / 1000.0f;
    float pulses_per_mm = 4096 / LEAD_SCREW_PITCH_MM;
    return static_cast<uint32_t>(mm * pulses_per_mm);
}
uint32_t MAE3Encoder::degreesToPulses(float degrees)
{
    float pulses_per_degree = 4096 / 360.0f;
    return static_cast<uint32_t>(degrees * pulses_per_degree);
}
float MAE3Encoder::pulsesToUm(float pulses)
{
    float mm = pulses * (LEAD_SCREW_PITCH_MM / 4096);
    return mm * 1000.0f;
}

bool MAE3Encoder::isStopped(uint32_t threshold_us) const
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
    gpio_isr_handler_add((gpio_num_t)_signalPin, (gpio_isr_t)(_encoderId == 0 ? interruptHandler0 : _encoderId == 1 ? interruptHandler1 : _encoderId == 2 ? interruptHandler2 : interruptHandler3), (void*)this);
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

    int     level = gpio_get_level((gpio_num_t)_signalPin);  // digitalRead(_signalPin);
    int     freq  = esp_clk_cpu_freq();                      //  240000000
    int64_t nowUs = static_cast<int64_t>(xthal_get_ccount()) / (freq / 1000000);

    _r_pulse.duration_us_count++;
    _interruptCount++;  // Increment total interrupt count

    if (level == LOW)
    {
        // Falling edge
        _lowEdgeCount++;  // Increment falling edge count
        _lowStart = nowUs;
    }
    else
    {
        // Rising edge
        _highEdgeCount++;  // Increment rising edge count
        _lowEnd      = nowUs;
        _r_pulse.low = _lowEnd - _lowStart;

        _lastRisingEdgeTime = _currentRisingEdge;
        _currentRisingEdge  = nowUs;

        if (_lastRisingEdgeTime != 0)
        {
            _r_pulse.pulse_Interval = _currentRisingEdge - _lastRisingEdgeTime;
            _r_pulse.high           = _r_pulse.pulse_Interval - _r_pulse.low;

            // Check each condition separately
            bool highOK   = (_r_pulse.high >= 1 && _r_pulse.high <= 4302);  // 4097
            bool lowOK    = (_r_pulse.low >= 1 && _r_pulse.low <= 4302);    // 4097
            bool totalOK  = (_r_pulse.pulse_Interval > 0);
            bool periodOK = ((_r_pulse.pulse_Interval) >= 3731 && (_r_pulse.pulse_Interval) <= 4545);
            bool overall  = highOK && lowOK && totalOK && periodOK;

            if (!overall)
            {
                return;
            }

            // Calculate current encoder reading
            uint32_t x_measured = ((_r_pulse.high * 4098) / _r_pulse.pulse_Interval) - 1;

            // Add reading to voting buffer
            _votingBuffer[_votingIndex] = x_measured;
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

            _dataReady = true;
        }
    }
}

// Get median width of high and low pulses
uint32_t MAE3Encoder::get_median_width_high() const
{
    std::array<uint32_t, PULSE_BUFFER_SIZE> temp;

    //  Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_h_buffer.data(), sizeof(uint32_t) * PULSE_BUFFER_SIZE);

    int countNonZero = std::count_if(temp.begin(), temp.end(), [](uint32_t v) { return v > 0; });
    if (countNonZero < 3)
        return _state.width_high;  // Not enough valid data yet and return the last valid value

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

uint32_t MAE3Encoder::get_median_width_low() const
{
    std::array<uint32_t, PULSE_BUFFER_SIZE> temp;

    //  Use memcpy for faster copy inside critical section
    memcpy(temp.data(), _width_l_buffer.data(), sizeof(uint32_t) * PULSE_BUFFER_SIZE);

    int countNonZero = std::count_if(temp.begin(), temp.end(), [](uint32_t v) { return v > 0; });
    if (countNonZero < 3)
        return _state.width_low;  // Not enough valid data yet and return the last valid value

    std::sort(temp.begin(), temp.end());
    return temp[PULSE_BUFFER_SIZE / 2];
}

// Voting mechanism implementation
votePair MAE3Encoder::getMostFrequentValue() const
{
    // Create a map to count occurrences of each value
    std::unordered_map<uint32_t, int> frequency_map;

    // Count occurrences of each value in the voting buffer
    for (size_t i = 0; i < VOTING_BUFFER_SIZE; ++i)
    {
        frequency_map[_votingBuffer[i]]++;
    }

    // Find the value with the highest frequency
    uint32_t most_frequent_value = _votingBuffer[0];  // Default to first value
    uint32_t max_frequency       = 0;

    for (const auto& pair : frequency_map)
    {
        if (pair.second > max_frequency)
        {
            max_frequency       = pair.second;
            most_frequent_value = pair.first;
        }
    }

    return {most_frequent_value, max_frequency};
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

// Interrupt counter methods implementation
uint32_t MAE3Encoder::getInterruptCount() const
{
    return _interruptCount;
}

uint32_t MAE3Encoder::getHighEdgeCount() const
{
    return _highEdgeCount;
}

uint32_t MAE3Encoder::getLowEdgeCount() const
{
    return _lowEdgeCount;
}

void MAE3Encoder::resetInterruptCounters()
{
    _interruptCount = 0;
    _highEdgeCount  = 0;
    _lowEdgeCount   = 0;
}

void MAE3Encoder::attachOnComplete(void (*callback)())
{
    _onComplete = callback;
}
void MAE3Encoder::handleStoreToMemory()
{
    if (_storageCompleteFlag)
    {
        _storageCompleteFlag = false;
        if (_onComplete)
            _onComplete();
    }
}
void MAE3Encoder::setStorageCompleteFlag(bool flag)
{
    _storageCompleteFlag = flag;
}
#ifndef PWM_ENCODER_CONFIG_H
#define PWM_ENCODER_CONFIG_H

// PWM Encoder Configuration for MAE3 12-bit
// Optimized settings for minimal ESP32 reading errors

// Timing Configuration
#define PWM_TIMEOUT_US    5000  // 5ms timeout for signal detection
#define PWM_MIN_PULSE_US  50    // Minimum valid pulse width (MAE3 spec)
#define PWM_MAX_PULSE_US  4097  // Maximum valid pulse width (MAE3 spec)
#define PWM_MAX_PERIOD_US 4098  // Maximum total period (MAE3 spec)

// Sampling Configuration
#define PWM_SAMPLES_COUNT   5     // Number of samples for averaging
#define PWM_SAMPLE_DELAY_US 100   // Delay between samples
#define PWM_FILTER_ALPHA    0.3f  // Low-pass filter coefficient (0.0-1.0)

// Interrupt Configuration
#define PWM_INTERRUPT_TIMEOUT_MS 100  // Timeout for interrupt-based reading
#define PWM_INTERRUPT_DELAY_US   10   // Delay in interrupt wait loop

// Validation Configuration
#define PWM_VALIDATION_ENABLED 1     // Enable/disable validation
#define PWM_OUTLIER_THRESHOLD  3.0f  // Standard deviation threshold for outliers

// Performance Configuration
#define PWM_USE_INTERRUPTS     1  // Use interrupt-based reading (1) or polling (0)
#define PWM_USE_MEDIAN_FILTER  1  // Use median filtering (1) or mean (0)
#define PWM_USE_LOWPASS_FILTER 1  // Use low-pass filtering (1) or raw values (0)

// Debug Configuration
#define PWM_DEBUG_ENABLED  1  // Enable debug output
#define PWM_ERROR_ANALYSIS 1  // Enable error analysis and statistics

// MAE3 Specific Constants
#define MAE3_FULL_SCALE   4096  // 12-bit resolution
#define MAE3_PWM_CONSTANT 4098  // PWM calculation constant
#define MAE3_FREQUENCY_HZ 250   // Typical PWM frequency
#define MAE3_PERIOD_US    4000  // Typical period in microseconds

// Error Estimation
#define PWM_ERROR_THRESHOLD_US  5  // Maximum acceptable timing error
#define PWM_JITTER_THRESHOLD_US 2  // Maximum acceptable jitter

// Hardware Configuration
#define PWM_PIN_1          36  // First encoder pin
#define PWM_PIN_2          39  // Second encoder pin
#define PWM_PULLUP_ENABLED 1   // Enable internal pull-up resistors

// Calibration Configuration
#define PWM_CALIBRATION_SAMPLES 100  // Number of samples for calibration
#define PWM_CALIBRATION_ENABLED 1    // Enable automatic calibration

#endif  // PWM_ENCODER_CONFIG_H
#ifndef DIR_MULTIPLEXER_H
#define DIR_MULTIPLEXER_H

#include <Arduino.h>

/**
 * @brief 4-to-1 DIR Signal Multiplexer for Stepper Motor Control
 *
 * This module interfaces with a 4-to-1 analog multiplexer (e.g., 74HC4052)
 * to route a single DIR signal from ESP32 to one of 4 stepper motor drivers.
 * Uses 2 select lines (S0, S1) to control channel selection.
 *
 * Pin Configuration:
 * - S0, S1: Channel selection pins (2 GPIO pins)
 * - DIR: Direction signal pin (1 GPIO pin connected to common Z)
 * - Y0-Y3: Output channels connected to motor drivers
 */
class DirMultiplexer
{
public:
    /**
     * @brief Constructor
     * @param s0Pin GPIO pin for S0 select line
     * @param s1Pin GPIO pin for S1 select line
     * @param dirPin GPIO pin for DIR signal (connected to common Z)
     */
    DirMultiplexer(uint8_t s0Pin, uint8_t s1Pin, uint8_t dirPin);

    /**
     * @brief Initialize the multiplexer
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Select a specific motor channel (0-3)
     * @param motorId Motor ID (0-3)
     * @return true if selection successful
     */
    bool selectMotor(uint8_t motorId);

    /**
     * @brief Set direction for the currently selected motor
     * @param direction true for forward, false for reverse
     */
    void setDirection(bool direction);

    /**
     * @brief Set direction for a specific motor (selects motor and sets direction)
     * @param motorId Motor ID (0-3)
     * @param direction true for forward, false for reverse
     * @return true if operation successful
     */
    bool setMotorDirection(uint8_t motorId, bool direction);

    /**
     * @brief Get currently selected motor ID
     * @return Current motor ID (0-3)
     */
    uint8_t getCurrentMotor() const;

    /**
     * @brief Get current direction state
     * @return true if forward, false if reverse
     */
    bool getCurrentDirection() const;

    /**
     * @brief Disable all outputs (high impedance state)
     */
    void disable();

    /**
     * @brief Test multiplexer functionality
     * @param printResults Print test results to Serial
     * @return true if test passed
     */
    bool test(bool printResults = true);

private:
    uint8_t _s0Pin;             ///< S0 select pin
    uint8_t _s1Pin;             ///< S1 select pin
    uint8_t _dirPin;            ///< DIR signal pin
    uint8_t _currentMotor;      ///< Currently selected motor (0-3)
    bool    _currentDirection;  ///< Current direction state
    bool    _initialized;       ///< Initialization flag

    /**
     * @brief Set select pins based on motor ID
     * @param motorId Motor ID (0-3)
     */
    void setSelectPins(uint8_t motorId);

    /**
     * @brief Validate motor ID
     * @param motorId Motor ID to validate
     * @return true if valid
     */
    bool isValidMotorId(uint8_t motorId) const;
};

#endif  // DIR_MULTIPLEXER_H
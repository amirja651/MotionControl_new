#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "MAE3Encoder.h"
#include "Pins.h"
#include "TMC5160Manager.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// Motor configuration constants
static constexpr int16_t STEPS_PER_REVOLUTION      = 200;                                            // Standard stepper motor
static constexpr int16_t MICROSTEPS_PER_STEP       = static_cast<int16_t>(DEFAULT_CURRENT_PANCAKE);  // 32 microsteps
static constexpr int32_t MICROSTEPS_PER_REVOLUTION = STEPS_PER_REVOLUTION * MICROSTEPS_PER_STEP;     // 6400 microsteps
static constexpr float   POSITION_ACCURACY_DEGREES = 0.1f;                                           // Target accuracy

static constexpr float PIXEL_SIZE_UM              = 5.2f;  // Size of each pixel in the camera (micrometers)
static constexpr float PIXEL_SIZE_MM              = (PIXEL_SIZE_UM * 1e-3f);
static constexpr float CAMERA_TO_MIRROR_LENGTH_MM = 195.0f;  // Distance from mirror to camera in millimeters (can be measured accurately)

// Movement types
enum class MovementType
{
    SHORT_RANGE,   // Few degrees
    MEDIUM_RANGE,  // Medium range
    LONG_RANGE     // Long range
};

// Control modes
enum class ControlMode
{
    OPEN_LOOP,    // Open-loop control only
    CLOSED_LOOP,  // Continuous closed-loop control
    HYBRID        // Read encoder once at start, then open-loop
};

// Movement command structure
struct MovementCommand
{
    uint8_t      motorId;       // Motor ID (0-3)
    float        targetAngle;   // Target angle in degrees (0-360)
    MovementType movementType;  // Type of movement
    bool         relative;      // True for relative movement, false for absolute
    uint32_t     priority;      // Command priority (higher = more important)
    ControlMode  controlMode;   // Control mode (open-loop, closed-loop, hybrid)
};

struct ConvertValuesFromDegrees
{
    int32_t PULSES_FROM_DEGREES;
    int32_t STEPS_FROM_DEGREES;
};

struct ConvertValuesFromPulses
{
    float   DEGREES_FROM_PULSES;
    int32_t STEPS_FROM_PULSES;
};

struct ConvertValuesFromSteps
{
    int32_t PULSES_FROM_STEPS;
    float   DEGREES_FROM_STEPS;
};

// Motor status structure
struct MotorStatus
{
    uint8_t      motorId;
    float        currentAngle;  // Current angle in degrees
    float        targetAngle;   // Target angle in degrees
    bool         isMoving;      // True if motor is currently moving
    bool         isEnabled;     // True if motor is enabled
    int32_t      currentSteps;  // Current position in steps
    int32_t      targetSteps;   // Target position in steps
    MovementType lastMovementType;
    uint32_t     movementStartTime;  // Time when movement started
    uint32_t     totalMovementTime;  // Total time for movement
    ControlMode  controlMode;        // Current control mode
    float        positionError;      // Position error in degrees
    float        encoderAngle;       // Encoder reading in degrees
};

// Position controller class
class PositionController
{
public:
    // Constructor
    PositionController(uint8_t motorId, TMC5160Manager& driver, uint16_t dirPin, uint16_t stepPin, uint16_t enPin, MAE3Encoder& encoder);
    PositionController(uint8_t motorId, TMC5160Manager& driver, uint16_t dirPin, uint16_t stepPin, uint16_t enPin);  // Constructor without encoder (for demo)
    ~PositionController();

    // Initialization
    bool begin();
    void setCurrentPosition(int32_t position);
    void enable();
    void disable();
    bool isEnabled() const;

    // Position control methods
    bool moveToAngle(float targetAngle, MovementType movementType = MovementType::MEDIUM_RANGE, ControlMode controlMode = ControlMode::OPEN_LOOP);
    bool moveRelative(float deltaAngle, MovementType movementType = MovementType::MEDIUM_RANGE, ControlMode controlMode = ControlMode::OPEN_LOOP);
    void stop();
    bool isMoving() const;
    bool isAtTarget() const;

    // Status and information
    float       getCurrentAngle() const;
    float       getTargetAngle() const;
    int32_t     getCurrentSteps() const;
    int32_t     getTargetSteps() const;
    MotorStatus getStatus();

    // Configuration
    void setMaxSpeed(float speedStepsPerSec);
    void setAcceleration(float accelerationStepsPerSec2);
    void setSpeedProfile(MovementType type, float maxSpeed, float acceleration);

    // Angle utilities
    static float wrapAngle(float angle);
    static float calculateShortestPath(float currentAngle, float targetAngle);

    // RTOS task management
    static void startPositionControlTask();
    static void stopPositionControlTask();
    static bool queueMovementCommand(const MovementCommand& command);

    ConvertValuesFromDegrees convertFromDegrees(float degrees, int32_t microsteps = DEFAULT_CURRENT_PANCAKE * 200, int32_t resolution = ENCODER_RESOLUTION) const;
    ConvertValuesFromPulses  convertFromPulses(int32_t pulses, int32_t microsteps = DEFAULT_CURRENT_PANCAKE * 200, int32_t resolution = ENCODER_RESOLUTION) const;
    ConvertValuesFromSteps   convertFromMSteps(int32_t steps, int32_t microsteps = DEFAULT_CURRENT_PANCAKE * 200, int32_t resolution = ENCODER_RESOLUTION) const;
    float                    calculateMotorAngleFromReference(float newPixel, float refPixel, float refMotorDeg);
    float                    getEncoderAngle();

private:
    // Hardware components
    TMC5160Manager&      _driver;
    mutable AccelStepper _stepper;  // Mutable to allow const member functions to call AccelStepper methods
    MAE3Encoder*         _encoder;  // Pointer to encoder for position feedback (can be nullptr)
    uint16_t             _dirPin;
    uint16_t             _stepPin;
    uint16_t             _enPin;
    uint8_t              _motorId;

    // State variables
    MotorStatus _status;
    bool        _enabled;
    bool        _initialized;
    ControlMode _controlMode;

    // Speed profiles for different movement types
    struct SpeedProfile
    {
        float maxSpeed;      // Steps per second
        float acceleration;  // Steps per second squared
    };
    SpeedProfile _speedProfiles[3];  // Indexed by MovementType

    // RTOS components
    static TaskHandle_t      _positionControlTaskHandle;
    static QueueHandle_t     _movementCommandQueue;
    static SemaphoreHandle_t _statusMutex;

public:
    static PositionController* _instances[4];  // Made public for global access
private:
    // Private methods
    void  updateStatus();
    void  configureSpeedProfile(MovementType type);
    void  setSpeedForMovement(MovementType type);
    bool  executeMovement(const MovementCommand& command);
    float calculateOptimalSpeed(float distance, MovementType type);

    // Control mode methods
    void        setControlMode(ControlMode mode);
    ControlMode getControlMode() const;
    bool        isClosedLoopEnabled() const;
    bool        isHybridModeEnabled() const;

    float calculatePositionError();
    void  applyClosedLoopCorrection();
    void  applyHybridModeCorrection();

    // RTOS task function
    static void positionControlTask(void* parameter);
    void        runPositionControl();
};

// Global functions for RTOS integration
void initializePositionControllers();
void startPositionControlSystem();
void stopPositionControlSystem();

#endif  // POSITION_CONTROLLER_H
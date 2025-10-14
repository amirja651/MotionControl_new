#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "DirMultiplexer_lean.h"
#include "MAE3Encoder_lean.h"
#include "Pins_lean.h"
#include "TMC5160Manager_lean.h"
#include "UnitConverter_lean.h"

// Motor configuration constants
static constexpr int32_t POSITION_ACCURACY_STEPS = 3;  // Target accuracy in steps (0.05° ≈ 3 steps)

// Movement types
enum class MovementType
{
    SHORT_RANGE,   // Few degrees
    MEDIUM_RANGE,  // Medium range
    LONG_RANGE     // Long range
};

// Distance-based movement types for precise control
enum class DistanceType
{
    NEGLIGIBLE,  // < 6 steps - ignored
    VERY_SHORT,  // 6 - 32 steps
    SHORT,       // 32 - 64 steps
    MEDIUM,      // 64 - 320 steps
    LONG,        // 320 - 640 steps
    VERY_LONG    // > 640 steps
};

// Control modes
enum class ControlMode : uint8_t
{
    OPEN_LOOP,  // Open-loop control only
    HYBRID      // Read encoder once at start, then open-loop
};

// Movement command structure
struct MovementCommand
{
    uint8_t      motorId;                // Motor ID (0-3)
    int32_t      targetSteps;            // Target position in steps
    MovementType movementType;           // Type of movement
    DistanceType distanceType;           // Distance-based type
    bool         relative;               // True for relative movement, false for absolute
    uint32_t     priority;               // Command priority (higher = more important)
    ControlMode  controlMode;            // Control mode (open-loop, hybrid)
    int32_t      movementDistanceSteps;  // Calculated movement distance in steps
};

// Motor status structure
struct MotorStatus
{
    uint8_t      motorId;
    int32_t      currentSteps;  // Current position in steps
    int32_t      targetSteps;   // Target position in steps
    bool         isMoving;      // True if motor is currently moving
    bool         isEnabled;     // True if motor is enabled
    MovementType lastMovementType;
    uint32_t     movementStartTime;   // Time when movement started
    uint32_t     totalMovementTime;   // Total time for movement
    ControlMode  controlMode;         // Current control mode
    int32_t      positionErrorSteps;  // Position error in steps
    int32_t      encoderSteps;        // Encoder reading in steps
};

// Position controller class
class PositionController
{
public:
    // Constructor
    PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin, uint16_t enPin, mae3::Mae3Encoder& encoderMae3);
    PositionController(uint8_t motorId, TMC5160Manager& driver, DirMultiplexer& dirMultiplexer, uint16_t stepPin,
                       uint16_t enPin);  // Constructor without encoder (for demo)
    ~PositionController();

    // Initialization
    bool begin();
    void setCurrentPosition(int32_t positionSteps);
    void enable();
    void disable();
    bool isEnabled() const;

    // Position control methods
    bool moveToSteps(int32_t targetSteps, MovementType movementType = MovementType::MEDIUM_RANGE, ControlMode controlMode = ControlMode::OPEN_LOOP);
    bool moveRelativeSteps(int32_t deltaSteps, MovementType movementType = MovementType::MEDIUM_RANGE, ControlMode controlMode = ControlMode::OPEN_LOOP);
    void stop();
    bool isMoving() const;
    bool isAtTarget() const;

    // Status and information
    int32_t     getCurrentSteps() const;
    int32_t     getTargetSteps() const;
    MotorStatus getMotorStatus();

    // Configuration
    void setMaxSpeed(float speedStepsPerSec);
    void setAcceleration(float accelerationStepsPerSec2);
    void setSpeedProfile(MovementType type, float maxSpeed, float acceleration);
    void setDirection(bool direction);

    // Distance-based control methods
    DistanceType calculateDistanceTypeSteps(int32_t distanceSteps);
    bool         isValidMovementDistanceSteps(int32_t distanceSteps);
    void         setDistanceBasedSpeedProfile(DistanceType distanceType);
    float        calculateOptimalSpeedForDistanceSteps(int32_t distanceSteps);
    float        calculateOptimalAccelerationForDistanceSteps(int32_t distanceSteps);
    void         configureSpeedForDistanceSteps(int32_t distanceSteps);

    // RTOS task management
    static void startPositionControlTask();
    static void stopPositionControlTask();
    static bool queueMovementCommand(const MovementCommand& command);

    int32_t getEncoderSteps();
    // EncoderState getEncoderState() const;

    void attachOnComplete(void (*cb)());
    void handleMovementComplete();
    void setMovementCompleteFlag(bool flag);

    MotorType getMotorType() const;

private:
    // Hardware components
    TMC5160Manager&      _driver;
    mutable AccelStepper _stepper;  // Mutable to allow const member functions to call AccelStepper methods
    mae3::Mae3Encoder*   _encoderMae3;
    DirMultiplexer&      _dirMultiplexer;  // Direction signal multiplexer
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

    // Distance-based speed profiles for precise control
    struct DistanceSpeedProfile
    {
        float   maxSpeed;                   // Steps per second
        float   acceleration;               // Steps per second squared
        int32_t decelerationDistanceSteps;  // Distance to start deceleration (steps)
    };
    DistanceSpeedProfile _distanceSpeedProfiles[6];  // Indexed by DistanceType

    // RTOS components
    static TaskHandle_t      _positionControlTaskHandle;
    static QueueHandle_t     _movementCommandQueue;
    static SemaphoreHandle_t _statusMutex;

public:
    static PositionController* _instances[4];  // Made public for global access

    // Control mode methods
    void setControlMode(ControlMode mode);

private:
    // Private methods
    void  updateStatus();
    void  configureSpeedProfile(MovementType type);
    void  setSpeedForMovement(MovementType type);
    bool  executeMovement(const MovementCommand& command);
    float calculateOptimalSpeedSteps(int32_t distanceSteps, MovementType type);

    // Control mode methods
    ControlMode getControlMode() const;
    bool        isHybridModeEnabled() const;

    void applyHybridModeCorrection();

    // RTOS task function
    static void positionControlTask(void* parameter);
    void        runPositionControl();

    // Optional movement complete callback
    volatile bool _movementCompleteFlag;
    void (*_onComplete)();
};

// Global functions for RTOS integration
void initializePositionControllers();
void startPositionControlSystem();
void stopPositionControlSystem();

#endif  // POSITION_CONTROLLER_H
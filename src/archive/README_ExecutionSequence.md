# Motion Control System - Execution Sequence

## Overview
This document explains the step-by-step execution sequence when a user sends a movement command to the motion control system.

## Movement Command Execution Flow

### 1. User Input Reception
```
User Input → Serial Interface → Command Processing
```

**Entry Points:**
- **Keyboard Input**: Direct key presses (Q, J, A-Z, etc.)
- **CLI Commands**: `motor -n 1 -p 45.0` format
- **Serial Commands**: Direct serial communication

### 2. Command Parsing & Validation
```
Serial Input → Command Parser → Parameter Validation → Motor Selection
```

**Sequence:**
1. `serialReadTask()` receives character input
2. `cli.parse(inputBuffer)` processes CLI commands
3. `setMotorId(motorIdStr)` selects target motor
4. Parameter validation (angle range, motor index)

### 3. Movement Command Creation
```
Validated Parameters → MovementCommand Structure → Queue Submission
```

**MovementCommand Structure:**
```cpp
struct MovementCommand {
    uint8_t motorId;        // Target motor (0-3)
    float targetAngle;      // Target angle in degrees
    MovementType movementType; // SHORT/MEDIUM/LONG_RANGE
    bool relative;          // Relative vs absolute movement
    uint32_t priority;      // Command priority
    bool closedLoop;        // Open/closed-loop control
};
```

### 4. Command Queuing
```
MovementCommand → RTOS Queue → Position Control Task
```

**Sequence:**
1. `positionController[currentIndex].moveToAngle(targetAngle, movementType, closedLoop)`
2. `PositionController::queueMovementCommand(command)`
3. `xQueueSend(_movementCommandQueue, &command, 0)` - Add to RTOS queue

### 5. Position Control Task Processing
```
RTOS Task → Command Dequeue → Movement Execution → Status Update
```

**Task Execution (Core 1, Priority 3):**
1. `positionControlTask()` - Main task loop (10ms period)
2. `xQueueReceive(_movementCommandQueue, &command, 0)` - Get command
3. `executeMovement(command)` - Process movement

### 6. Movement Planning
```
Target Angle → Angle Wrapping → Shortest Path → Microstep Conversion
```

**Sequence:**
1. `wrapAngle(targetAngle)` - Normalize to 0-360°
2. `calculateShortestPath(currentAngle, targetAngle)` - Find optimal path
3. `degreesToMicrosteps(targetAngle)` - Convert to stepper units
4. `setSpeedForMovement(movementType)` - Configure speed profile

### 7. Motor Configuration
```
Speed Profile → AccelStepper Configuration → Driver Settings
```

**Configuration:**
1. `configureSpeedProfile(movementType)` - Set speed/acceleration
2. `_stepper.setMaxSpeed(maxSpeed)` - Configure stepper
3. `_stepper.setAcceleration(acceleration)` - Set acceleration
4. `_stepper.moveTo(targetMicrosteps)` - Set target position

### 8. Movement Execution
```
AccelStepper → TMC5160 Driver → Motor Movement → Position Feedback
```

**Execution Loop:**
1. `_stepper.run()` - Execute one step
2. `_stepper.distanceToGo()` - Check if movement complete
3. `updateStatus()` - Update position status
4. Repeat until target reached

### 9. Closed-Loop Control (If Enabled)
```
Encoder Reading → Position Error → Correction → Motor Adjustment
```

**Closed-Loop Sequence:**
1. `getEncoderAngle()` - Read encoder position
2. `calculatePositionError()` - Calculate position error
3. `applyClosedLoopCorrection()` - Apply correction
4. Adjust stepper target if needed

### 10. Status Monitoring & Completion
```
Position Tracking → Status Updates → Completion Detection → User Feedback
```

**Monitoring:**
1. `getStatus()` - Get current motor status
2. `isMoving()` - Check if movement active
3. `isAtTarget()` - Check if target reached
4. Status reporting to user

## Detailed Method Call Sequence

### For Keyboard Command 'Q' (Move to Origin):
```
1. serialReadTask() - Receives 'Q' key
2. loadOriginPosition(currentIndex) - Get stored origin
3. positionController[currentIndex].moveToAngle(targetAngle, movementType, false)
4. PositionController::moveToAngle()
5. PositionController::wrapAngle()
6. PositionController::calculateShortestPath()
7. PositionController::setSpeedForMovement()
8. PositionController::queueMovementCommand()
9. xQueueSend(_movementCommandQueue, &command, 0)
10. positionControlTask() - Processes command
11. PositionController::executeMovement()
12. _stepper.moveTo(targetMicrosteps)
13. _stepper.run() - Movement execution loop
14. PositionController::updateStatus()
15. Movement completion detection
```

### For CLI Command 'motor -n 1 -p 45.0':
```
1. serialReadTask() - Receives serial input
2. cli.parse(inputBuffer) - Parse CLI command
3. setMotorId("1") - Select motor 1
4. positionController[currentIndex].moveToAngle(45.0, movementType, false)
5. PositionController::moveToAngle()
6. PositionController::wrapAngle(45.0) → 45.0°
7. PositionController::calculateShortestPath(current, 45.0)
8. PositionController::setSpeedForMovement(MEDIUM_RANGE)
9. PositionController::queueMovementCommand()
10. xQueueSend(_movementCommandQueue, &command, 0)
11. positionControlTask() - RTOS task processes command
12. PositionController::executeMovement()
13. _stepper.moveTo(targetMicrosteps)
14. _stepper.run() - Execute movement
15. PositionController::updateStatus()
16. Movement completion
```

## RTOS Task Scheduling

### Core 0 (Serial Processing):
- **Task**: `serialReadTask()`
- **Priority**: 2
- **Period**: 100ms
- **Function**: Command reception and parsing

### Core 1 (Position Control):
- **Task**: `positionControlTask()`
- **Priority**: 3
- **Period**: 10ms
- **Function**: Movement execution and control

## Timing Characteristics

### Command Processing:
- **Serial parsing**: <1ms
- **Command queuing**: <1ms
- **Movement planning**: <1ms
- **Total command processing**: <3ms

### Movement Execution:
- **Task period**: 10ms
- **Step execution**: <100μs per step
- **Status update**: <1ms
- **Position accuracy**: ±0.1°

### System Response:
- **Command to movement start**: <15ms
- **Real-time responsiveness**: 10ms task period
- **Position feedback**: Every 10ms

## Error Handling Sequence

### Connection Issues:
```
1. TMC5160Manager::testConnection() - Check driver connection
2. Driver status validation
3. Error reporting to user
4. Command rejection if driver not available
```

### Movement Errors:
```
1. PositionController::moveToAngle() - Validate parameters
2. Angle range checking (0-360°)
3. Motor enable state verification
4. Error reporting and command rejection
```

### Encoder Errors (Closed-loop):
```
1. MAE3Encoder::processPWM() - Read encoder
2. Encoder state validation
3. Position error calculation
4. Fallback to open-loop if encoder fails
```

## Memory and Resource Management

### Queue Management:
- **Command queue size**: 10 commands
- **Queue overflow**: Commands rejected
- **Memory allocation**: Static allocation

### Task Management:
- **Stack size**: 4KB per task
- **Stack overflow**: Detection enabled
- **Watchdog**: 5-second timeout

This execution sequence ensures reliable, real-time motion control with proper error handling and resource management. 
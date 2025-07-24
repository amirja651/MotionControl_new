# Distance-Based Speed Control System

## Overview
The MotionControl_2 system now features an advanced distance-based speed control system that automatically optimizes motor speed and acceleration based on movement distance. This ensures smooth, precise motion without overshooting or vibrations, especially for small movements.

## Distance Ranges and Speed Profiles

### Distance Classification
The system classifies movements into 6 categories based on distance:

| Distance Range | Type | Speed Profile | Purpose |
|----------------|------|---------------|---------|
| < 0.1° | NEGLIGIBLE | Ignored | Prevents unnecessary movements |
| 0.1° - 0.5° | VERY_SHORT | 500 steps/s, 1000 steps/s² | Ultra-precise positioning |
| 0.5° - 1° | SHORT | 1000 steps/s, 2000 steps/s² | Precise positioning |
| 1° - 5° | MEDIUM | 2000 steps/s, 4000 steps/s² | Balanced speed/precision |
| 5° - 10° | LONG | 4000 steps/s, 8000 steps/s² | Moderate speed |
| > 10° | VERY_LONG | 8000 steps/s, 16000 steps/s² | High speed for long distances |

### Speed Profile Structure
```cpp
struct DistanceSpeedProfile
{
    float maxSpeed;              // Steps per second
    float acceleration;          // Steps per second squared
    float decelerationDistance;  // Distance to start deceleration (degrees)
};
```

## Implementation Details

### 1. Distance Validation
```cpp
bool isValidMovementDistance(float distance)
{
    return distance >= 0.1f;  // Ignore movements smaller than 0.1°
}
```

### 2. Distance Type Calculation
```cpp
DistanceType calculateDistanceType(float distance)
{
    if (distance < 0.1f)
        return DistanceType::NEGLIGIBLE;
    else if (distance < 0.5f)
        return DistanceType::VERY_SHORT;
    else if (distance < 1.0f)
        return DistanceType::SHORT;
    else if (distance < 5.0f)
        return DistanceType::MEDIUM;
    else if (distance < 10.0f)
        return DistanceType::LONG;
    else
        return DistanceType::VERY_LONG;
}
```

### 3. Optimal Speed Calculation
The system uses linear interpolation within each distance range for fine-tuned speed control:

```cpp
float calculateOptimalSpeedForDistance(float distance)
{
    DistanceType distanceType = calculateDistanceType(distance);
    float baseSpeed = _distanceSpeedProfiles[index].maxSpeed;
    float speedMultiplier = 1.0f;
    
    switch (distanceType)
    {
        case DistanceType::VERY_SHORT:
            // 0.1° - 0.5°: Linear interpolation (0.5x to 1.0x)
            speedMultiplier = 0.5f + (distance - 0.1f) * 0.5f / 0.4f;
            break;
        case DistanceType::SHORT:
            // 0.5° - 1°: Linear interpolation (0.7x to 1.0x)
            speedMultiplier = 0.7f + (distance - 0.5f) * 0.3f / 0.5f;
            break;
        // ... other cases
    }
    
    return baseSpeed * speedMultiplier;
}
```

### 4. Optimal Acceleration Calculation
Similar to speed, acceleration is also fine-tuned within each range:

```cpp
float calculateOptimalAccelerationForDistance(float distance)
{
    // Lower acceleration for shorter distances to prevent overshooting
    // Higher acceleration for longer distances for faster movement
    // Linear interpolation within each range
}
```

## Usage Examples

### 1. Basic Movement
```cpp
// The system automatically determines optimal speed/acceleration
positionController.moveToAngle(45.0f, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);
```

### 2. Small Movement (0.3°)
```cpp
// Automatically uses VERY_SHORT profile with reduced speed
positionController.moveToAngle(currentAngle + 0.3f, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);
```

### 3. Large Movement (50°)
```cpp
// Automatically uses VERY_LONG profile with higher speed
positionController.moveToAngle(50.0f, MovementType::MEDIUM_RANGE, ControlMode::OPEN_LOOP);
```

## Testing the System

### Keyboard Commands
- **'T'**: Display distance-based speed profiles for test distances
- **'1'-'7'**: Test specific movement distances (0.2°, 0.4°, 0.5°, 1°, 5°, 10°, 50°)

### CLI Commands
```bash
# All movements automatically use distance-based speed control
motor -n 1 -p 0.3    # Very short movement (0.3°)
motor -n 1 -p 2.5    # Medium movement (2.5°)
motor -n 1 -p 25.0   # Long movement (25°)
```

## Benefits

### 1. Precision for Small Movements
- Movements < 1° use reduced speed and acceleration
- Prevents overshooting and vibrations
- Ensures accurate final positioning

### 2. Efficiency for Large Movements
- Movements > 10° use higher speed profiles
- Faster completion times
- Maintains precision through proper deceleration

### 3. Automatic Optimization
- No manual speed/acceleration selection required
- System automatically chooses optimal parameters
- Consistent behavior across all movement distances

### 4. Smooth Motion
- Linear interpolation within distance ranges
- Gradual speed transitions
- No sudden speed changes

## Technical Specifications

### Hardware Requirements
- TMC5160 stepper drivers
- 200 steps/revolution motors
- 256 microsteps (51,200 microsteps/revolution)
- Position accuracy: ±0.1°

### Performance Characteristics
- **Minimum Movement**: 0.1° (negligible movements ignored)
- **Maximum Speed**: 16,000 steps/s (for very long movements)
- **Minimum Speed**: 500 steps/s (for very short movements)
- **Position Resolution**: 0.007° per microstep

### Safety Features
- Movement distance validation
- Automatic speed/acceleration limits
- Overshoot prevention through reduced acceleration
- Negligible movement filtering

## Integration with Existing System

### Backward Compatibility
- All existing movement commands work unchanged
- MovementType parameter still accepted (for compatibility)
- Distance-based control is automatic and transparent

### Enhanced Features
- Automatic distance calculation and validation
- Optimal speed/acceleration selection
- Improved precision for small movements
- Better performance for large movements

### Control Modes
- **Open-Loop**: Distance-based speed control
- **Closed-Loop**: Distance-based speed + encoder feedback
- **Hybrid**: Distance-based speed + initial encoder reading

## Future Enhancements

### Potential Improvements
1. **Adaptive Profiles**: Learn optimal speeds based on motor performance
2. **Load Compensation**: Adjust profiles based on mechanical load
3. **Temperature Compensation**: Modify profiles based on motor temperature
4. **Custom Profiles**: User-defined speed profiles for specific applications

### Advanced Features
1. **Predictive Deceleration**: Start deceleration before reaching target
2. **S-Curve Acceleration**: Smoother acceleration profiles
3. **Multi-Axis Coordination**: Synchronized multi-axis movements
4. **Path Optimization**: Optimal paths for complex movements

## Troubleshooting

### Common Issues
1. **Movement Too Slow**: Check if distance is correctly calculated
2. **Overshooting**: Verify acceleration settings for distance range
3. **Vibrations**: Ensure speed/acceleration are appropriate for distance
4. **Ignored Movements**: Check if movement distance is < 0.1°

### Debug Information
- Use 'T' command to view speed profiles
- Use 'L' command to monitor position status
- Check serial output for distance and speed information
- Monitor encoder readings for closed-loop control

This distance-based speed control system provides the foundation for precise, efficient, and reliable motion control across all movement distances while maintaining backward compatibility with existing code. 
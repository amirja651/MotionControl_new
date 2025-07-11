#include "Pins.h"
#include "PositionController.h"
#include "TMC5160Manager.h"
#include <Arduino.h>


// Demo program for position control functionality
// This demonstrates:
// 1. Short-range movements (few degrees)
// 2. Medium-range movements
// 3. Long-range movements
// 4. Angle wrapping and shortest path calculation
// 5. RTOS task scheduling

class PositionControlDemo
{
private:
    TMC5160Manager     driver;
    PositionController positionController;
    uint8_t            motorId;

    // Demo sequence tracking
    uint8_t  currentDemoStep;
    uint32_t lastDemoTime;
    bool     demoRunning;

    // Demo sequences
    struct DemoSequence
    {
        float        angles[10];
        MovementType types[10];
        const char*  descriptions[10];
        uint8_t      count;
    };

    DemoSequence shortRangeDemo;
    DemoSequence mediumRangeDemo;
    DemoSequence longRangeDemo;
    DemoSequence angleWrappingDemo;

public:
    PositionControlDemo(uint8_t motorId)
        : driver(motorId, DriverPins::CS[motorId]),
          positionController(motorId, driver, DriverPins::DIR[motorId], DriverPins::STEP[motorId], DriverPins::EN[motorId]),
          motorId(motorId),
          currentDemoStep(0),
          lastDemoTime(0),
          demoRunning(false)
    {
        initializeDemoSequences();
    }

    bool begin()
    {
        // Initialize driver
        if (!driver.begin())
        {
            Serial.printf("[Demo] Failed to initialize driver for motor %d\n", motorId + 1);
            return false;
        }

        if (!driver.testConnection(true))
        {
            Serial.printf("[Demo] Driver connection test failed for motor %d\n", motorId + 1);
            return false;
        }

        // Configure driver
        driver.configureDriver_All_Motors(true);

        // Initialize position controller
        if (!positionController.begin())
        {
            Serial.printf("[Demo] Failed to initialize position controller for motor %d\n", motorId + 1);
            return false;
        }

        // Enable position controller
        positionController.enable();

        Serial.printf("[Demo] Position control demo initialized for motor %d\n", motorId + 1);
        return true;
    }

    void runDemo()
    {
        if (!demoRunning)
        {
            demoRunning     = true;
            currentDemoStep = 0;
            lastDemoTime    = millis();
            Serial.println(F("\n=== Position Control Demo Started ==="));
            Serial.println(F("Demo will run through various movement types:"));
            Serial.println(F("1. Short-range movements (1-10 degrees)"));
            Serial.println(F("2. Medium-range movements (10-90 degrees)"));
            Serial.println(F("3. Long-range movements (90-360 degrees)"));
            Serial.println(F("4. Angle wrapping demonstrations"));
            Serial.println(F("=====================================\n"));
        }

        // Check if current movement is complete
        if (!positionController.isMoving())
        {
            // Wait a bit before next movement
            if (millis() - lastDemoTime > 2000)  // 2 second delay between movements
            {
                executeNextDemoStep();
            }
        }
    }

    void stopDemo()
    {
        demoRunning = false;
        positionController.stop();
        Serial.println(F("\n=== Position Control Demo Stopped ==="));
    }

    void printStatus()
    {
        MotorStatus status = positionController.getStatus();
        Serial.printf("[Demo] Motor %d: Current=%.2f°, Target=%.2f°, Moving=%s, Enabled=%s\n", motorId + 1, status.currentAngle, status.targetAngle,
                      status.isMoving ? "YES" : "NO", status.isEnabled ? "YES" : "NO");
    }

private:
    void initializeDemoSequences()
    {
        // Short-range demo (1-10 degrees)
        shortRangeDemo.count     = 5;
        shortRangeDemo.angles[0] = 5.0f;
        shortRangeDemo.angles[1] = -3.0f;
        shortRangeDemo.angles[2] = 8.0f;
        shortRangeDemo.angles[3] = -5.0f;
        shortRangeDemo.angles[4] = 0.0f;
        for (int i = 0; i < shortRangeDemo.count; i++)
        {
            shortRangeDemo.types[i]        = MovementType::SHORT_RANGE;
            shortRangeDemo.descriptions[i] = "Short-range movement";
        }

        // Medium-range demo (10-90 degrees)
        mediumRangeDemo.count     = 4;
        mediumRangeDemo.angles[0] = 45.0f;
        mediumRangeDemo.angles[1] = 90.0f;
        mediumRangeDemo.angles[2] = 135.0f;
        mediumRangeDemo.angles[3] = 180.0f;
        for (int i = 0; i < mediumRangeDemo.count; i++)
        {
            mediumRangeDemo.types[i]        = MovementType::MEDIUM_RANGE;
            mediumRangeDemo.descriptions[i] = "Medium-range movement";
        }

        // Long-range demo (90-360 degrees)
        longRangeDemo.count     = 4;
        longRangeDemo.angles[0] = 270.0f;
        longRangeDemo.angles[1] = 360.0f;
        longRangeDemo.angles[2] = 180.0f;
        longRangeDemo.angles[3] = 0.0f;
        for (int i = 0; i < longRangeDemo.count; i++)
        {
            longRangeDemo.types[i]        = MovementType::LONG_RANGE;
            longRangeDemo.descriptions[i] = "Long-range movement";
        }

        // Angle wrapping demo
        angleWrappingDemo.count     = 6;
        angleWrappingDemo.angles[0] = 350.0f;  // Should go to 350° (not -10°)
        angleWrappingDemo.angles[1] = 10.0f;   // Should go to 10° (shortest path)
        angleWrappingDemo.angles[2] = 370.0f;  // Should wrap to 10°
        angleWrappingDemo.angles[3] = -10.0f;  // Should wrap to 350°
        angleWrappingDemo.angles[4] = 720.0f;  // Should wrap to 0°
        angleWrappingDemo.angles[5] = 0.0f;    // Back to 0°
        for (int i = 0; i < angleWrappingDemo.count; i++)
        {
            angleWrappingDemo.types[i]        = MovementType::MEDIUM_RANGE;
            angleWrappingDemo.descriptions[i] = "Angle wrapping test";
        }
    }

    void executeNextDemoStep()
    {
        DemoSequence* currentSequence = nullptr;
        const char*   sequenceName    = "";

        // Determine which sequence to run based on current step
        if (currentDemoStep < shortRangeDemo.count)
        {
            currentSequence = &shortRangeDemo;
            sequenceName    = "Short-Range";
        }
        else if (currentDemoStep < shortRangeDemo.count + mediumRangeDemo.count)
        {
            currentSequence = &mediumRangeDemo;
            sequenceName    = "Medium-Range";
        }
        else if (currentDemoStep < shortRangeDemo.count + mediumRangeDemo.count + longRangeDemo.count)
        {
            currentSequence = &longRangeDemo;
            sequenceName    = "Long-Range";
        }
        else if (currentDemoStep < shortRangeDemo.count + mediumRangeDemo.count + longRangeDemo.count + angleWrappingDemo.count)
        {
            currentSequence = &angleWrappingDemo;
            sequenceName    = "Angle-Wrapping";
        }
        else
        {
            // Demo complete
            Serial.println(F("\n=== Position Control Demo Complete ==="));
            Serial.println(F("All movement types and angle wrapping demonstrated successfully!"));
            demoRunning = false;
            return;
        }

        // Calculate index within current sequence
        uint8_t sequenceIndex = currentDemoStep;
        if (currentDemoStep >= shortRangeDemo.count)
            sequenceIndex -= shortRangeDemo.count;
        if (currentDemoStep >= shortRangeDemo.count + mediumRangeDemo.count)
            sequenceIndex -= mediumRangeDemo.count;
        if (currentDemoStep >= shortRangeDemo.count + mediumRangeDemo.count + longRangeDemo.count)
            sequenceIndex -= longRangeDemo.count;

        // Execute movement
        float        targetAngle  = currentSequence->angles[sequenceIndex];
        MovementType movementType = currentSequence->types[sequenceIndex];

        Serial.printf("[Demo] %s Demo - Step %d: Moving to %.1f° (Type: %d)\n", sequenceName, currentDemoStep + 1, targetAngle, static_cast<int>(movementType));

        bool success = positionController.moveToAngle(targetAngle, movementType);
        if (!success)
        {
            Serial.printf("[Demo] Failed to execute movement to %.1f°\n", targetAngle);
        }

        currentDemoStep++;
        lastDemoTime = millis();
    }
};

// Global demo instance (for motor 0)
PositionControlDemo* demo = nullptr;

// Demo control functions
void startPositionControlDemo()
{
    if (demo == nullptr)
    {
        demo = new PositionControlDemo(0);  // Use motor 0 for demo
        if (!demo->begin())
        {
            Serial.println(F("[Demo] Failed to initialize demo"));
            delete demo;
            demo = nullptr;
            return;
        }
    }

    demo->runDemo();
}

void stopPositionControlDemo()
{
    if (demo != nullptr)
    {
        demo->stopDemo();
        delete demo;
        demo = nullptr;
    }
}

void printDemoStatus()
{
    if (demo != nullptr)
    {
        demo->printStatus();
    }
    else
    {
        Serial.println(F("[Demo] No demo running"));
    }
}
#pragma once

#ifndef MAIN_H
    #define MAIN_H

    #include "Defines.h"
    #include "MAE3Encoder.h"
    #include "MotorSpeedController.h"
    #include "Pins.h"
    #include "SystemDiagnostics.h"
    #include "TMC5160Manager.h"
    #include <Arduino.h>
    #include <CLIManager.h>
    #include <SPI.h>
    #include <SimpleCLI.h>
    #include <TMCStepper.h>
    #include <esp_task_wdt.h>
    #include <memory>

    #define _MIN_SPEED       20
    #define _FINE_MOVE_SPEED 12
    #define _MAX_SPEED       200

struct MotorContext
{
    float   currentPosition;
    float   error;
    int32_t currentPositionPulses;
    int32_t targetPositionPulses;
    int32_t errorPulses;
};

std::unique_ptr<TMC5160Manager>       driver[NUM_MOTORS]    = {nullptr};
std::unique_ptr<MotorSpeedController> motor[NUM_MOTORS]     = {nullptr};
std::unique_ptr<MAE3Encoder>          encoder[NUM_ENCODERS] = {nullptr};

// Task handles
TaskHandle_t encoderUpdateTaskHandle = NULL;
TaskHandle_t motorUpdateTaskHandle   = NULL;
TaskHandle_t serialReadTaskHandle    = NULL;

static constexpr uint32_t SPI_CLOCK = 1000000;  // 1MHz SPI clock

static uint8_t currentIndex            = 0;  // Current driver index
static int32_t lastPulse[NUM_ENCODERS] = {0};

static float targetPosition[NUM_ENCODERS] = {0};

static bool driverEnabled[NUM_MOTORS]               = {false};
static bool newTargetpositionReceived[NUM_ENCODERS] = {false};
static bool firstTime                               = true;

    // Command history support
    #define HISTORY_SIZE 10
static String commandHistory[HISTORY_SIZE];
static int    historyCount = 0;
static int    historyIndex = -1;  // -1 means not navigating

bool motorMoving[NUM_MOTORS] = {false};

static float lastSpeed[NUM_MOTORS] = {0.0f};

// Static variable to store total distance for speed profile calculations
static float initialTotalDistance[NUM_MOTORS] = {0.0f};

// Static variable to store high water mark
static unsigned int highWaterMark = 0;

void encoderUpdateTask(void* pvParameters);
void motorUpdateTask(void* pvParameters);
void serialReadTask(void* pvParameters);

bool         isValidNumber(const String& str);
void         setTargetPosition(String targetPos);
void         setMotorId(String motorId);
float        getMotorPosition();
MotorContext getMotorContext();
void         linearMotorUpdate();
void         rotaryMotorUpdate();
int          calculateSteppedSpeed(float progressPercent, int minSpeed, int maxSpeed, float segmentSizePercent);
void         demonstrateSpeedProfile();
void         motorStopAndSavePosition(String callerFunctionName);
void         clearLine();
void         printSerial();
void         printMotorStatus();
void         resetMotorState(uint8_t motorId);

#endif
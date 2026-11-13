#ifndef GLOBAL_H
#define GLOBAL_H

/**
 * @file globals.h
 * @brief Global variables and constants used across multiple modules.
 */

#define NUM_CP 14         // Number of checkpoints allowed in path planning
#define TOTAL_INDEX 99    // Max number of steps in path planning
#define NUM_PID 4         // Number of unique PID profiles

// === Voltage Thresholds for Analog Button Detection ===
// These values are based on the voltage divider circuit for multiple buttons on a single analog pin.
#define BTN1_MAX    60     // Voltage threshold max for Button 1
#define BTN2_MIN    500
#define BTN2_MAX    700    // Range for Button 2
#define BTN3_MIN    800
#define BTN3_MAX    900   // Range for Button 3
#define IDLE_MIN    1005   // Minimum value indicating no button press

#define MAX_DURATION 5000  // Max button press duration (in ms) to be considered valid

// === Button Identifiers ===
#define BTN_1 1
#define BTN_2 2
#define BTN_3 3
#define BTN_4 4
#define BTN_5 5
#define BTN_6 6

// === Struct: Button Press Info ===
struct ButtonInfo {
  int id = 0;                  // Identified button number (mapped using analog voltage)
  unsigned long duration = 0; // Time in ms the button was held
  int pressCount = 0;         // Used for pattern recognition like double-taps
};

// === Struct: PID Controller Parameters ===
typedef struct {
  byte Kp;       // Proportional gain
  byte Kd;       // Derivative gain
  byte Ki;       // Integral gain
  byte PMax;     // Output saturation upper bound
  int PMin;      // Output saturation lower bound (can be negative)
} dataPID;

// === Struct: System Configuration ===
typedef struct {
  uint16_t LIMIT_VALUE_SENSOR[16];  // Threshold values for 16 IR sensors
  int speed;                        // Default robot speed
  byte thisCheckPoint;             // Currently active checkpoint
  byte checkPoint[NUM_CP];         // Logical checkpoint to physical mapping
  byte stopIndex;                  // Index where the robot should stop
  byte sensor_sensitivity;         // Adjusts detection sensitivity
  byte lineColor;                  // Expected line color (e.g., black = 0, white = 1)
  byte numPID;                     // Active PID profile index
} dataSetting;

// === Struct: One Path Step in Path Planning ===
typedef struct {
  byte action;                     // Action type (e.g., FOLLOW_LINE, TURN, STOP)
  unsigned int sensorBitValue[2];  // Sensor pattern bitmask (start/end condition)
  byte modeSensor;                 // Interpretation logic: XOR, OR, ONLY, etc.
  int L, R, D;                     // Left speed, Right speed, Delay or Distance
  byte SA;                         // Speed used during PID-controlled movement
  unsigned int TA;                 // Duration (in ms) to apply PID control
  byte lineColor;                  // Line color expected for this step
  byte numPID;                     // Which PID setting to use
} dataIndex;

// === Global Variables Shared Across Modules ===
extern ButtonInfo currentButtonInfo;
extern dataSetting setting;
extern unsigned int preDefinedThreshold[16];
extern bool previouslyCalibrated;

extern dataIndex ramIndexData[TOTAL_INDEX];
extern dataPID listPID[NUM_PID];

extern uint16_t reading[16];  // Stores real-time analog sensor readings

#endif // GLOBAL_H

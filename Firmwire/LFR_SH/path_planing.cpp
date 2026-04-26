#include <Wire.h>
#include "config.h"
#include "init.h"
#include "globals.h"
#include "functions.h"
#include <Arduino.h>
#include "path_planing.h"


/* ================================
   === PID CONTROL VARIABLES =====
   ================================ */

/**
 * Real-time encoder error, used for balancing left and right motor speeds.
 * Calculated as the difference in encoder counts or movement between motors.
 */
int error1 = 0;

/**
 * Stores the previous encoder error value for derivative calculations in PID.
 */
int lastError1 = 0;

/**
 * Calculated PWM speed values for the left and right motors after PID correction.
 */
int LMotorSpeed = 0;
int RMotorSpeed = 0;


/* ================================
   ===== SENSOR READING ARRAY =====
   ================================ */

// Uncomment and use this array to store raw sensor readings if needed
// uint16_t reading[16];  // Stores raw analog readings from 16 sensors (e.g. IR reflectance sensors)


/* ================================
   === PID CONTROL FOR SENSORS ===
   ================================ */

/**
 * PID terms for sensor-based line following control:
 * These variables help compute the control output for motors based on line position errors.
 */
double P = 0;           // Proportional term (based on current error)
double D = 0;           // Derivative term (rate of change of error)
double error = 0;       // Current line following error from sensor feedback
double lastError = 0;   // Previous error, for derivative calculation

/**
 * Timestamp of last PID computation.
 * Used with millis() for timing the update interval between PID calculations.
 */
unsigned long lastProcess = 0;

/**
 * Tunnel index where the robot path enters a tunnel segment.
 * Used for special handling during navigation (like speed adjustment or sensor filtering).
 */
int tunnelIndex = 12;


/* ============================================
   === PID SETTINGS INITIALIZATION FUNCTION ===
   ============================================ */

/**
 * @brief initializePIDSettings()
 * 
 * Sets up multiple PID parameter profiles for different robot behaviors.
 * This allows switching PID gains dynamically based on the navigation scenario.
 * 
 * PID profiles control motor speed outputs based on error correction:
 *  - Kp: Proportional gain
 *  - Ki: Integral gain (not used here, set to zero)
 *  - Kd: Derivative gain
 *  - PMax: Max output PWM value (upper motor speed limit)
 *  - PMin: Min output PWM value (lower motor speed limit or reverse)
 */
void initializePIDSettings() {
  // PID_0: Default line following with medium response and stability
  setPID(PID_0, 6, 0, 35, 255, -120);

  // PID_1: More aggressive for sharper turns
  setPID(PID_1, 8, 0, 40, 255, -120);

  // PID_2: Quick sharp response for tight mazes
  setPID(PID_2, 9, 0, 45, 255, -120);

  // PID_3: High precision with strong derivative damping for narrow or high speed paths
  setPID(PID_3, 10, 0, 25, 255, -200);

  /*
  // Optional PID tuning profiles, uncomment if needed
  setPID(PID_4, 12, 0, 20, 255, -200);
  setPID(PID_5, 15, 0, 25, 255, -250);
  */
}


/* ==================================================================
   === CHECKPOINTS INITIALIZATION FUNCTION ==========================
   ================================================================== */

/**
 * @brief initializeCheckPoints()
 * 
 * Registers checkpoints used for robot path recovery and resuming.
 * Each checkpoint corresponds to a certain path index the robot passes.
 * Also defines any path indexes where the robot should temporarily stop.
 * 
 * Important:
 * - Each checkpoint requires at least 4 prior indexes for completion.
 * - Checkpoint is considered complete after reaching the defined index.
 */
void initializeCheckPoints() {
  // Register checkpoints with corresponding indexes
  setCheckPoint(CP_0, 0);
  setCheckPoint(CP_1, 1);
  setCheckPoint(CP_2, 2);
  setCheckPoint(CP_3, 3);
  setCheckPoint(CP_4, 4);
  setCheckPoint(CP_5, 5);
  setCheckPoint(CP_6, 6);
  setCheckPoint(CP_7, 7);
  setCheckPoint(CP_8, 8);
  setCheckPoint(CP_9, 9);
  setCheckPoint(CP_10, 10);
  setCheckPoint(CP_11, 11);
  setCheckPoint(CP_12, 12);
  setCheckPoint(CP_13, 13);
  setCheckPoint(CP_14, 14);
  setCheckPoint(CP_15, 15);
  setCheckPoint(CP_16, 16);
  setCheckPoint(CP_17, 17);
  setCheckPoint(CP_18, 18);
  setCheckPoint(CP_19, 19);
  setCheckPoint(CP_20, 20);
  setCheckPoint(CP_21, 21);
  setCheckPoint(CP_22, 22);
  setCheckPoint(CP_23, 23);
  setCheckPoint(CP_24, 24);
  setCheckPoint(CP_25, 25);
  setCheckPoint(CP_26, 26);
  setCheckPoint(CP_27, 27);
  setCheckPoint(CP_28, 28);
  setCheckPoint(CP_29, 29);
  setCheckPoint(CP_30, 30);
  setCheckPoint(CP_31, 31);
  setCheckPoint(CP_32, 32);
  setCheckPoint(CP_33, 33);
  setCheckPoint(CP_34, 34);
  setCheckPoint(CP_35, 35);
  setCheckPoint(CP_36, 36);
  setCheckPoint(CP_37, 37);
  setCheckPoint(CP_38, 38);
  setCheckPoint(CP_39, 39);
  setCheckPoint(CP_40, 40);
  setCheckPoint(CP_41, 41);
  setCheckPoint(CP_42, 42);
  setCheckPoint(CP_43, 43);
  setCheckPoint(CP_44, 44);
  setCheckPoint(CP_45, 45);
  setCheckPoint(CP_46, 46);
  setCheckPoint(CP_47, 47);
  setCheckPoint(CP_48, 48);
  setCheckPoint(CP_49, 49);
  setCheckPoint(CP_50, 50);

  // Define a temporary stop at index 14 for tasks like obstacle checking or pauses
  stopAtIndex(14);
}


/* ==========================================================
   === PATH SEQUENCE INITIALIZATION FUNCTION ===============
   ========================================================== */

/**
 * @brief initializePathSequence()
 * 
 * Defines the robot’s step-by-step movement and control instructions.
 * Each `setIndex()` call sets the behavior at a specific path index:
 *  - Sensor pattern expected (e.g. all sensors, right, leftMiddle)
 *  - Motion decision (go straight, turn left/right, specific commands)
 *  - Motor speeds (PWM values for left and right motors)
 *  - Duration or timing for blind turns or specific actions (milliseconds)
 *  - Line color to track (black or white line)
 *  - Speed amplification or base speed setting
 *  - Allowed time for PID or other special movements
 *  - PID tuning profile to use for this segment
 * 
 * This setup is modular to allow complex path following, sharp turns, and speed control.
 */
void initializePathSequence() {
  // Start of path sequence definitions

  // Index 0: Move forward with default PID_3 settings at moderate speed
  setIndex(0, all, MOTION, 60, 60, 60, blackLine, 160, 100, PID_3);

  // Index 1: Turn right using sensors on right side, max speed 160
  setIndex(1, right, turnRight, 160, blackLine, 160, 0, PID_3);

  // Index 2: Turn left at left middle sensors with 100 speed and 250ms timing
  setIndex(2, leftMiddle, turnLeft, 100, blackLine, 160, 250, PID_3);

  // Index 3: Turn right using right middle sensors, 100 speed, 40ms delay
  setIndex(3, rightMiddle, turnRight, 100, blackLine, 160, 40, PID_3);

  // Index 4: Specific motion using left middle sensors with 80 speed both motors
  setIndex(4, leftMiddle, SPECIFIC, 80, 80, 100, blackLine, 160, 0, PID_3);

  // Index 5: Motion with differential speeds (-80 left, 80 right) for a pivot turn
  setIndex(5, all, MOTION, -80, 80, 80, blackLine, 160, 60, PID_3);

  // Index 6: Specific command with right middle sensors, 80 speed both motors
  setIndex(6, rightMiddle, SPECIFIC, 80, 80, 100, blackLine, 140, 0, PID_3);

  // Index 7: Opposite pivot turn from index 5 (80 left, -80 right)
  setIndex(7, all, MOTION, 80, -80, 80, blackLine, 160, 50, PID_3);

  // Index 8: Repeat specific motion at right middle sensors
  setIndex(8, rightMiddle, SPECIFIC, 80, 80, 100, blackLine, 140, 0, PID_3);

  // Index 9: Another pivot turn (-80 left, 80 right)
  setIndex(9, all, MOTION, -80, 80, 80, blackLine, 140, 60, PID_3);

  // Index 10: Straight motion from left middle sensors at 140 speed and 200ms timing
  setIndex(10, leftMiddle, straight, 140, blackLine, 140, 200, PID_3);

  // Index 11: Turn left at left sensors, speed 100, 30ms delay
  setIndex(11, left, turnLeft, 100, blackLine, 140, 30, PID_3);

  // Index 12: Straight on white line at 100 speed, 60ms delay
  setIndex(12, empty, straight, 100, whiteLine, 160, 60, PID_3);

  // Index 13: Turn right at right sensors, speed 100, 2000ms delay (long turn)
  setIndex(13, right, turnRight, 100, blackLine, 160, 2000, PID_3);

  // Index 14: Straight motion when all sensors detect line, speed 80, no delay
  setIndex(14, allHit, straight, 80, blackLine, 160, 0, PID_3);

  // Additional path steps can be added here as needed.
  // Older or commented-out path steps can be uncommented for extension or testing.
}

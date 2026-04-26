#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

/* ================================
   ======= SPEED CONSTANTS ========
   ================================ */

/**
 * @brief Maximum and minimum PWM speed values for motors during PID control.
 * 
 * maxSpeed1: Maximum forward speed PWM value (positive).
 * minSpeed1: Minimum speed PWM value (can be negative for reverse).
 */
#define maxSpeed1  250      // Max PWM speed for motors (forward)
#define minSpeed1 -130      // Min PWM speed for motors (reverse)

/**
 * @brief Base speed used when moving straight using encoder feedback.
 */
#define straightBaseSpeed 100  // Base PWM speed for straight encoder control


/* ================================
   ===== PID CONTROL VARIABLES =====
   ================================ */

/**
 * @brief PID coefficients used for encoder-based motor correction.
 */
#define Kp1  1    // Proportional gain for encoder PID control
#define Kd1  1    // Derivative gain for encoder PID control

// Variables used for PID error calculation
extern int error1;         // Current encoder error (difference between left and right)
extern int lastError1;     // Previous encoder error (for derivative computation)

// Motor speed variables calculated by PID control (PWM values)
extern int LMotorSpeed;    // Speed command for Left Motor
extern int RMotorSpeed;    // Speed command for Right Motor


/* ================================
   ===== SENSOR READING ARRAY =====
   ================================ */

/**
 * @brief Array holding raw sensor readings (e.g., 16 IR or reflectance sensors).
 */
// extern uint16_t reading[16];  // Uncomment and define in implementation


/* ================================
   ===== PID CONTROL FOR SENSORS ===
   ================================ */

/**
 * @brief Variables for PID control based on sensor feedback (line-following).
 */
extern double P;            // Proportional term from sensor error
extern double D;            // Derivative term (rate of change of error)
extern double error;        // Current line following error (sensor-based deviation)
extern double lastError;    // Previous error value (for derivative calculation)

/**
 * @brief Timing variables to manage PID update intervals.
 */
extern unsigned long lastProcess;  // Last PID process timestamp (millis())
extern int tunnelIndex;             // Index indicating when tunnel section starts


/* ================================
   ===== CHECKPOINT DEFINITIONS ====
   ================================ */

/**
 * @brief Total number of checkpoints for recovery and restart.
 * Can be adjusted dynamically (e.g., by buttons).
 */
#define NUM_CP 31

/**
 * @brief PID Controller profile indexes.
 * Allows switching between different PID profiles.
 */
#define PID_0 0
#define PID_1 1
#define PID_2 2
#define PID_3 3
// #define PID_4 4   // Uncomment for extra profiles
// #define PID_5 5

/**
 * @brief Checkpoint identifiers for robot restart positions.
 * Each checkpoint is a saved state or position for recovery.
 * Only NUM_CP checkpoints are actively used; others can be extended.
 */
#define CP_0 0
#define CP_1 1
#define CP_2 2
#define CP_3 3
#define CP_4 4
#define CP_5 5
#define CP_6 6
#define CP_7 7
#define CP_8 8
#define CP_9 9
#define CP_10 10
#define CP_11 11
#define CP_12 12
#define CP_13 13
#define CP_14 14
#define CP_15 15
#define CP_16 16
#define CP_17 17
#define CP_18 18
#define CP_19 19
#define CP_20 20
#define CP_21 21
#define CP_22 22
#define CP_23 23
#define CP_24 24
#define CP_25 25
#define CP_26 26
#define CP_27 27
#define CP_28 28
#define CP_29 29
#define CP_30 30
#define CP_31 31
#define CP_32 32
#define CP_33 33
#define CP_34 34
#define CP_35 35
#define CP_36 36
#define CP_37 37
#define CP_38 38
#define CP_39 39
#define CP_40 40
#define CP_41 41
#define CP_42 42
#define CP_43 43
#define CP_44 44
#define CP_45 45
#define CP_46 46
#define CP_47 47
#define CP_48 48
#define CP_49 49
#define CP_50 50


/* ================================
   ===== MOTION ACTION CONSTANTS ===
   ================================ */

/**
 * @brief Motion types for robot commands.
 * 
 * MOTION: Open-loop motion (without sensor feedback).
 * ACTION: Closed-loop motion using sensors (PID control).
 * SPECIFIC: Special movements triggered under certain conditions.
 */
#define MOTION 1
#define ACTION 0
#define SPECIFIC 2


/* ================================
   ===== PREDEFINED MOVEMENT MACROS
   ================================ */

/**
 * @brief Common robot movement commands defined by action type and motor speeds.
 * 
 * Format: ACTION_TYPE, LeftMotorSpeed, RightMotorSpeed
 */
#define turnLeft    ACTION, -100, 220   // Sharp left turn: left motor reverse, right motor forward
#define turnRight   ACTION, 220, -100   // Sharp right turn: right motor reverse, left motor forward
#define straight    ACTION, 80, 80      // Move forward straight with moderate speed
#define stop        ACTION, 0, 0        // Stop both motors


/* ================================
   ===== LINE COLOR DEFINITIONS =====
   ================================ */

/**
 * @brief Define the line color the robot is expected to follow.
 */
#define blackLine 0   // Follow black line on white surface
#define whiteLine 1   // Follow white line on black surface


/* ================================
   ===== SENSOR MODE CONSTANTS =====
   ================================ */

/**
 * @brief Defines whether to use sensor input or not.
 */
#define ACTION_NOT_USE_SENSOR 1  // Robot acts without sensor input (open loop)
#define ACTION_USE_SENSOR    0   // Robot acts with sensor input (closed loop)

/**
 * @brief Logical modes for sensor pattern matching.
 */
#define OR    0   // At least one sensor must be active
#define EQUAL 1   // All sensors must match a pattern exactly
#define XOR   2   // Multiple valid paths (e.g., crossroads)
#define ONLY  3   // Exactly one specific pattern matched (forced direction)


/* ================================
   ===== SENSOR PATTERN MACROS =====
   ================================ */

/**
 * @brief Common sensor bitmask definitions and their modes.
 * 
 * Format: Mask0, Mask1, Mode
 */
#define all            0b1111111111111111, 0b0000000000000000, OR     // Any sensor active
#define allHit         0b1111111111111111, 0b0000000000000000, EQUAL  // All sensors active
#define empty          0b0000000000000000, 0b0000000000000000, EQUAL  // No sensors active

#define left           0b1110000000000000, 0b0000000000000000, OR     // Any left sensors active
#define right          0b0000000000000111, 0b0000000000000000, OR     // Any right sensors active

#define leftRight      0b1111100000000000, 0b0000000000011111, XOR    // Both left and right active (multiple paths)
#define leftMiddle     0b1111000000000000, 0b0000011111100000, XOR    // Left and center sensors active
#define rightMiddle    0b0000000000001111, 0b0000011111100000, XOR    // Right and center sensors active

#define rightAngle     0b0000000000001111, 0b0000011111100000, XOR    // Sharp right elbow detected
#define leftAngle      0b1111000000000000, 0b0000011111100000, XOR    // Sharp left elbow detected

#define leftO          0b1110000000000000, 0b0000011111100000, ONLY   // Left-only path (force left turn)
#define rightO         0b0000000000000111, 0b0000011111100000, ONLY   // Right-only path (force right turn)


/* ================================
   ===== FUNCTION DECLARATIONS =====
   ================================ */

/**
 * @brief Initializes various PID controller profiles used by the robot.
 * 
 * This sets different PID gain sets for different behaviors or path sections.
 */
void initializePIDSettings();

/**
 * @brief Initializes all checkpoints used for robot recovery and restarts.
 * 
 * Checkpoints represent safe resume points along the path.
 */
void initializeCheckPoints();

/**
 * @brief Defines the entire sequence of movements and sensor logic for the path.
 * 
 * Each segment is set using setIndex() with specific sensor triggers, motor speeds,
 * PID profiles, and special actions like blind turns or timed motions.
 */
void initializePathSequence();


#endif // PATH_PLANNING_H

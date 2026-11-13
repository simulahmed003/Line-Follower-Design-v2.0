#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

// ====== Definitions ======
#define whiteLine 1   // White line detection mode
#define blackLine 0   // Black line detection mode

// ======================== Function Prototypes ========================


///// SONAR ///////

// #define NUM_SONARS 3
// extern const int sonarPins[NUM_SONARS];
// extern volatile float distances[NUM_SONARS];
// extern volatile bool readingReady;
// extern volatile unsigned long echoStartTime;
// extern int maxDistance;
// extern int currentSensor;
// extern unsigned long maxEchoDuration;

// void echoISR();
// void updateSonarReading();
// void sonarInit();


///// SONAR ///////
void followLine (int dataSensor);

void setCheckPoint(byte num, byte index);

/**
 * @brief Set speed for both left and right motors.
 * 
 * @param L Speed for left motor (-255 to +255).
 * @param R Speed for right motor (-255 to +255).
 */
void setMotor(int16_t L, int16_t R);

/**
 * @brief Detects a button press using voltage divider logic on an analog pin.
 *        Updates `currentButtonInfo` global variable.
 * 
 * @param pin Analog pin to read button press from.
 * @return true if button press detected, false otherwise.
 */
bool detectButtonPress(int pin);

/**
 * @brief Reads all 16 sensors via multiplexer.
 * 
 * @return 16-bit bitfield where each bit represents sensor activation.
 */
uint16_t readSensor();

/**
 * @brief Adjusts the sensor threshold level globally.
 * 
 * @param sensitivity Value from 0 to 100 affecting detection threshold.
 */
void setSensorSensivity(byte sensitivity);

/**
 * @brief Runs full sensor calibration routine by collecting min/max values.
 *        Stores result in `setting.LIMIT_VALUE_SENSOR`.
 */
void calibrateSensor();

/**
 * @brief Generates a short buzzer beep to indicate success.
 */
void success();

/**
 * @brief Checks if a button is physically pressed (digital read).
 * 
 * @param buttonPin Pin number connected to the button.
 * @return true if button is pressed, false otherwise.
 */
bool isButtonPressed(int buttonPin);

/**
 * @brief Reads digital button press with debouncing and press counting.
 * 
 * @param buttonPin Button pin to read.
 * @return Integer code or count representing the button status.
 */
int read_button(int buttonPin);

/**
 * @brief Populates a step (index) in the path planning array.
 * 
 * @param index Index of the step to be configured.
 * @param sensor0 First sensor mask (e.g., expected line pattern).
 * @param sensor1 Second sensor mask (used with XOR/ONLY logic).
 * @param modeSensor Logic mode (XOR, OR, ONLY, etc.).
 * @param action Action type to perform (e.g., TURN, FOLLOW_LINE).
 * @param L Left motor speed.
 * @param R Right motor speed.
 * @param D Duration or delay in ms or encoder ticks.
 * @param lineColor Expected line color (black or white).
 * @param SA Base speed during PID motion.
 * @param TA Duration to apply PID control.
 * @param numPID PID profile index to use.
 */
void setIndex (
  byte index,
  unsigned int sensor0,
  unsigned int sensor1,
  byte modeSensor,
  byte action,
  int L,
  int R,
  unsigned int D,
  byte lineColor,
  byte SA,
  unsigned int TA,
  byte numPID
);

/**
 * @brief Marks the stopping index in path planning.
 * 
 * @param index Index at which the robot should stop.
 */
void stopAtIndex(byte index);

/**
 * @brief Sets PID controller values for a given PID profile.
 * 
 * @param num PID profile index to configure.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param PMax Maximum output cap.
 * @param PMin Minimum output cap.
 */
void setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin);

#endif // FUNCTIONS_H

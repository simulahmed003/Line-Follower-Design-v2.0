#include <Wire.h>                  // I2C communication library
#include "config.h"                // Contains pin definitions, constants, and macros
#include "init.h"                  // Initialization functions (e.g., motors, mux, buzzer)
#include "globals.h"               // Global variables used across files
#include "functions.h"            // Core logic and utility function declarations
#include "path_planing.h"       // Algorithm and checkpoint/index planning
#include <SoftwareSerial.h>       // Library for software-based serial communication
#include "tunnel.h"



/**
 * Pins used for single-pin ultrasonic sonar sensors:
 * sonarPins[0] => PB12 (Right)
 * sonarPins[1] => PB13 (Mid)
 * sonarPins[2] => PA14 (Left)
 */
const int sonarPins[NUM_SONARS] = {PB12, PB13, PA14}; 

// Global sensor readings and flags
volatile float distances[NUM_SONARS] = {0};           // Stores distance values in cm
volatile bool readingReady = true;                    // Indicates if a valid echo reading is available
volatile unsigned long echoStartTime = 0;             // Timestamp for echo signal start

// Sensor logic control
int maxDistance = 200;                                // Maximum measurable distance (used as timeout fallback)
int currentSensor = 0;                                // Index of the current sonar sensor being triggered
unsigned long maxEchoDuration = 10000;                // Maximum echo wait time in microseconds (~10ms)

// Interrupt Service Routine: called on rising/falling edge of echo pin
void echoISR() {
  if (digitalRead(sonarPins[currentSensor]) == HIGH) {
    // Echo pulse started
    echoStartTime = micros();
  } else {
    // Echo pulse ended, calculate duration
    unsigned long echoEndTime = micros();
    unsigned long duration = echoEndTime - echoStartTime;
    distances[currentSensor] = (float)duration * 0.0343 / 2.0; // Convert microseconds to centimeters
    readingReady = true;
  }
}


// Main sonar logic: triggers one sensor at a time in non-blocking mode
void updateSonarReading() {
  // Proceed if previous reading is complete or timeout occurred
  if (readingReady || (micros() - echoStartTime > maxEchoDuration)) {
    
    // Timeout fallback: set max distance if no echo was detected
    if (!readingReady && (micros() - echoStartTime > maxEchoDuration)) {
      distances[currentSensor] = maxDistance;
    }

    // Move to next sensor in a cyclic manner
    currentSensor = (currentSensor + 1) % NUM_SONARS;

    // Disable other sonar pins and set them LOW
    for (int i = 0; i < NUM_SONARS; i++) {
      if (i != currentSensor) {
        detachInterrupt(digitalPinToInterrupt(sonarPins[i]));
        pinMode(sonarPins[i], OUTPUT);
        digitalWrite(sonarPins[i], LOW);
      }
    }

    // Trigger ultrasonic pulse on the current sonar
    detachInterrupt(digitalPinToInterrupt(sonarPins[currentSensor]));
    pinMode(sonarPins[currentSensor], OUTPUT);
    digitalWrite(sonarPins[currentSensor], LOW);
    delayMicroseconds(2);
    digitalWrite(sonarPins[currentSensor], HIGH);
    delayMicroseconds(10);
    digitalWrite(sonarPins[currentSensor], LOW);

    // Switch pin to input to receive echo and reattach interrupt
    pinMode(sonarPins[currentSensor], INPUT);
    attachInterrupt(digitalPinToInterrupt(sonarPins[currentSensor]), echoISR, CHANGE);

    // Reset flag for next echo detection
    readingReady = false;
  }
}


void sonarInit(){
  
  // Setup all sonar pins as input initially and attach interrupts
  for (int i = 0; i < NUM_SONARS; i++) {
    pinMode(sonarPins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(sonarPins[i]), echoISR, CHANGE);
  }
}





int wall_limit = 20;
int obstacle_limit = 20;

int turn90_delay = 270;
// extern int errorWall;
// extern int lastErrorWall;
// extern bool forwardTurn;



void wall_follow_1() {
    const int velocity = 80;               // Base motor speed
    const int Kp = 5;                      // Proportional gain
    const int Kd = 3;                      // Derivative gain
    const int midSet = 25;                // Ideal distance to wall
    const int maxCorrection = 80;         // Max PID output to avoid sharp turns

    int errorWall = 0;
    int lastErrorWall = 0;
    int p = 0;                             // PID output
    bool forwardTurn = false;             // Indicates if the robot should check for line after a forward obstacle turn

    while (true) {
        updateSonarReading();  // Non-blocking update of all ultrasonic sensors

        // Check for forward obstacle and turn 90° left if necessary
        if ((distances[1] < obstacle_limit) && !forwardTurn) {
            setMotor(0, 0);
            delay(100);
            success(); success();          // Signal obstacle detection
            setMotor(-velocity, velocity); // Perform left turn
            delay(turn90_delay);
            setMotor(0, 0);
            delay(100);
            forwardTurn = true;           // After turning, start checking for line
            continue;
        }

        // If in forward-turn mode, check for line detection and exit if line is found
        if (forwardTurn) {
            int dataBit = readSensor();
            if (dataBit != 0) {
                setMotor(0, 0);            // Stop motors
                break;                     // Exit tunnel-following loop
            }
        }

        // Compute wall-following error based on left/right sensor readings
        bool leftWall = distances[0] < wall_limit;
        bool rightWall = distances[2] < wall_limit;

        if (leftWall && rightWall) {
            errorWall = distances[0] - distances[2];  // Track the midpoint
        } else if (leftWall) {
            errorWall = distances[0] - midSet;        // Left wall only
        } else if (rightWall) {
            errorWall = midSet - distances[2];        // Right wall only
        }

        // If no wall is detected on either side, move forward slowly
        if (!leftWall && !rightWall) {
            setMotor(50, 50);                         // Soft forward motion
        } else {
            // PID Control for wall following
            p = Kp * errorWall + Kd * (lastErrorWall - errorWall);
            lastErrorWall = errorWall;

            // Limit correction output
            if (p < -maxCorrection) p = -maxCorrection;
            if (p > maxCorrection) p = maxCorrection;

            // Adjust motor speeds to steer based on error
            setMotor(velocity - p, velocity + p);
        }

        delay(20);  // Small delay to stabilize readings
    }
}



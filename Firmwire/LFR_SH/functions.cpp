#include <Arduino.h>
#include "config.h"
#include "init.h"
#include "globals.h"  // Assumes global variables like `reading[16]` and `setting` defined there
#include "functions.h"
#include "path_planing.h"

// ====== Buzzer 'Tit Tit' Function ======
void success() {
  digitalWrite(buzzerPin, HIGH);
  delay(25);
  digitalWrite(buzzerPin, LOW);
}

// ====== Checks if a Button is Pressed (with pull-up logic) ======

// ======================== Button Reading via Analog Voltage ========================
// For up to 3 buttons per analog pin using voltage dividers
// Uses shared state: `currentButtonInfo` global variable

// ====== Hardware Definitions ======
// #define SW0 PA0
// #define SW1 PA1

// // ====== Voltage Thresholds (Adjust as per your resistors) ======
// #define BTN1_MAX 100
// #define BTN2_MIN 400
// #define BTN2_MAX 500
// #define BTN3_MIN 600
// #define BTN3_MAX 700
// #define IDLE_MIN 800

// #define MAX_DURATION 5000  // Max time to consider a long press (in ms)

// // ====== Button Identifiers ======
// #define BTN_1 1
// #define BTN_2 2
// #define BTN_3 3
// #define BTN_4 4
// #define BTN_5 5
// #define BTN_6 6

// // ====== Struct to Hold Button Info ======
// struct ButtonInfo {
//   int id = 0;                  // Button ID (1-6)
//   unsigned long duration = 0; // Press duration (ms)
//   int pressCount = 0;         // Sequential press count
// };

// // ====== Global Button Info ======
// ButtonInfo currentButtonInfo ={0, 0, 0};


// ====== Function to Detect Button Press and Update Global Struct ======
bool detectButtonPress(int pin) {
  int analogValue = analogRead(pin);

  // No valid button press if idle
  if (analogValue > IDLE_MIN) return false;

  int buttonId = 0;
  if (analogValue < BTN1_MAX) {
    buttonId = (pin == SW0) ? BTN_1 : BTN_4;
  } else if (analogValue >= BTN2_MIN && analogValue <= BTN2_MAX) {
    buttonId = (pin == SW0) ? BTN_2 : BTN_5;
  } else if (analogValue >= BTN3_MIN && analogValue <= BTN3_MAX) {
    buttonId = (pin == SW0) ? BTN_3 : BTN_6;
  } else {
    return false;  // Unknown or invalid reading
  }

  // === Press Duration Measurement ===
  unsigned long pressStart = millis();
  while (true) {
    int value = analogRead(pin);
    if ((buttonId == BTN_1 || buttonId == BTN_4) && value >= BTN1_MAX) break;
    if ((buttonId == BTN_2 || buttonId == BTN_5) && !(value >= BTN2_MIN && value <= BTN2_MAX)) break;
    if ((buttonId == BTN_3 || buttonId == BTN_6) && !(value >= BTN3_MIN && value <= BTN3_MAX)) break;


    if (millis() - pressStart > MAX_DURATION) break;
    delay(1);
  }
  unsigned long duration = millis() - pressStart;

  // === Update Global Button Info ===
  if (currentButtonInfo.id == buttonId) {
    currentButtonInfo.pressCount++;
  } else {
    currentButtonInfo.pressCount = 1;
  }

  currentButtonInfo.id = buttonId;
  currentButtonInfo.duration = duration;

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

// motor control
void setMotor(int16_t L, int16_t R) {
  R = -R;
  if (L > 0) {
    digitalWrite(dirL, 0);
  }
  else {
    L = L + 255;
    digitalWrite(dirL, 1);
  }
  analogWrite(pwmL, L);
  if (R > 0) {
    digitalWrite(dirR, 0);
  }
  else {
    R = R + 255;
    digitalWrite(dirR, 1);
  }
  analogWrite(pwmR, R);
}


// ====== Sensor Reading Function (16-ch Analog MUX) ======
uint16_t readSensor() {
  uint16_t dataSensorBit = 0;

  // MUX channel to sensor index and selector mapping
  const static uint8_t muxSequence[16][2] = {
    {0,  0b0000}, {1,  0b0001}, {3,  0b0011}, {2,  0b0010},
    {6,  0b0110}, {7,  0b0111}, {5,  0b0101}, {4,  0b0100},
    {12, 0b1100}, {13, 0b1101}, {15, 0b1111}, {14, 0b1110},
    {10, 0b1010}, {11, 0b1011}, {9,  0b1001}, {8,  0b1000}
  };

  for (byte i = 0; i < 16; i++) {
    // Set MUX selector pins
    digitalWrite(s3, (muxSequence[i][1] >> 3) & 1);
    digitalWrite(s2, (muxSequence[i][1] >> 2) & 1);
    digitalWrite(s1, (muxSequence[i][1] >> 1) & 1);
    digitalWrite(s0, muxSequence[i][1] & 1);

    delayMicroseconds(5);  // Allow signal to settle
    // delay(1500);
    reading[muxSequence[i][0]] = analogRead(sig);  // Read from sensor
  }

  // Convert sensor readings into binary pattern
  for (int i = 0; i < 16; i++) {
    if (reading[i] < setting.LIMIT_VALUE_SENSOR[i]) {
      bitSet(dataSensorBit, 15 - i);  // MSB = leftmost sensor
    }
  }

  // Invert if line color is white
  if (setting.lineColor == whiteLine) {
    dataSensorBit = ~dataSensorBit & 0xFFFF;
  }

  return dataSensorBit;
}

// ====== Set Sensor Sensitivity (0-100) ======
void setSensorSensivity(byte sensitivity) {
  setting.sensor_sensitivity = sensitivity;
}

// ====== Calibrate All 16 Sensors ======
void calibrateSensor() {
  unsigned int minVal[16], maxVal[16] = {};  // Arrays for min/max values

  // Initialize min values to max ADC range
  for (int i = 0; i < 16; i++) minVal[i] = 4095;

  // Calibration loop until button2 is pressed
  while (1) {
    readSensor();
    for (byte i = 0; i < 16; i++) {
      if (reading[i] > maxVal[i]) maxVal[i] = reading[i];
      if (reading[i] < minVal[i]) minVal[i] = reading[i];
    }

    if (detectButtonPress(SW0)) {
      success();  // Sound buzzer
      break;
    }
  }

  // Calculate threshold values for each sensor
  for (int i = 0; i < 16; i++) {
    uint16_t range = maxVal[i] - minVal[i];
    float thresholdPercent = (float)setting.sensor_sensitivity / 100.0;

    setting.LIMIT_VALUE_SENSOR[i] = minVal[i] + (range * (1.0 - thresholdPercent));
  }
}



///////////// PID //////////////////////////////////////////


void followLine (int dataSensor) {                           //following a basic line(no intersection)
          double deltaTime = (millis() - lastProcess) / 1000.0; //***********************
          lastProcess = millis();
          switch (dataSensor) {
            case 0b0000000110000000: error = 0;    break;                          // middle position
            
            case 0b0000001110000000: error = 1;    break;                         //positive error
            case 0b0000000100000000: error = 1;    break; 
            case 0b0000001100000000: error = 2;    break;
            case 0b0000011100000000: error = 3;    break;
            case 0b0000001000000000: error = 3;    break;
            case 0b0000011000000000: error = 4;    break;
            case 0b0000111000000000: error = 5;    break;
            case 0b0000010000000000: error = 5;    break;
            case 0b0000110000000000: error = 6;    break;
            case 0b0001110000000000: error = 7;    break;
            case 0b0000100000000000: error = 7;    break;
            case 0b0001100000000000: error = 8;    break;
            case 0b0011100000000000: error = 9;    break;
            case 0b0001000000000000: error = 9;    break;
            case 0b0011000000000000: error = 10;    break;
            case 0b0111000000000000: error = 12;    break;
            case 0b0010000000000000: error = 12;    break;
            case 0b0110000000000000: error = 14;    break;
            case 0b1110000000000000: error = 15;    break;
            case 0b1111000000000000: error = 15;    break;
            case 0b0100000000000000: error = 15;    break;
            case 0b1100000000000000: error = 16;    break;
            case 0b1000000000000000: error = 17;    break;
  
            case 0b0000000111000000: error = -1;    break;                          //negetive error
            case 0b0000000010000000: error = -1;    break; 
            case 0b0000000011000000: error = -2;    break;
            case 0b0000000011100000: error = -3;    break;
            case 0b0000000001000000: error = -3;    break;
            case 0b0000000001100000: error = -4;    break;
            case 0b0000000001110000: error = -5;    break;
            case 0b0000000000100000: error = -5;    break;
            case 0b0000000000110000: error = -6;    break;
            case 0b0000000000111000: error = -7;    break;
            case 0b0000000000010000: error = -7;    break;
            case 0b0000000000011000: error = -8;    break;
            case 0b0000000000011100: error = -9;    break;
            case 0b0000000000001000: error = -9;    break;
            case 0b0000000000001100: error = -10;    break;
            case 0b0000000000001110: error = -12;    break;
            case 0b0000000000000100: error = -12;    break;
            case 0b0000000000000110: error = -14;    break;
            case 0b0000000000000111: error = -15;    break;
            case 0b0000000000001111: error = -15;    break;
            case 0b0000000000000010: error = -15;    break;
            case 0b0000000000000011: error = -16;    break;
            case 0b0000000000000001: error = -17;    break;
          }

          P = error * (double) listPID[setting.numPID].Kp;                                                   //PID is here
          D = (error - lastError) * (double) listPID[setting.numPID].Kd / deltaTime;

          double rateError = error - lastError;
          lastError = error;
          int moveVal = (int)P + (int)D;                                 // Integral part is not used . You can explore the role of integral part in line follower robot
          int moveLeft = setting.speed - moveVal;
          int moveRight = setting.speed + moveVal;
          if (moveLeft < listPID[setting.numPID].PMin)  moveLeft = listPID[setting.numPID].PMin;
          if (moveLeft > listPID[setting.numPID].PMax)  moveLeft = listPID[setting.numPID].PMax;
          if (moveRight < listPID[setting.numPID].PMin)  moveRight = listPID[setting.numPID].PMin;
          if (moveRight > listPID[setting.numPID].PMax)  moveRight = listPID[setting.numPID].PMax;
          setMotor(moveLeft, moveRight);
        }

//////////////////////////////////////////////////////////////////////////


// Sets a single behavior step for path planning at the given index.
// This function populates the ramIndexData[] array with all the motion and control parameters needed at a certain step.
void setIndex (
  byte index,                  // The index in the ramIndexData array where the settings will be stored
  unsigned int sensor0,        // First sensor condition (usually a bitmask representing which sensors should be active)
  unsigned int sensor1,        // Second sensor condition (used depending on sensor mode: XOR, ONLY, etc.)
  byte modeSensor,             // The sensor interpretation mode (e.g., XOR, OR, ONLY, etc.)
  byte action,                 // The action to perform at this step (e.g., MOTION, TURN, STOP)
  int L,                       // Speed value for the left motor
  int R,                       // Speed value for the right motor
  unsigned int D,              // Delay or movement duration (usually in milliseconds or encoder ticks)
  byte lineColor,              // The expected line color (GARIS_HITAM = black line, GARIS_PUTIH = white line)
  byte SA,                     // Maximum base speed used for PID control (SA = speed applied during PID phase)
  unsigned int TA,             // Duration in milliseconds to apply PID-based line following after this step
  byte numPID                  // Index of the PID setting to use (e.g., PID_0 to PID_5)
) {
  ramIndexData[index].sensorBitValue[0] = sensor0;    // Store first sensor bit pattern
  ramIndexData[index].sensorBitValue[1] = sensor1;    // Store second sensor bit pattern
  ramIndexData[index].L = L;                          // Set left motor speed
  ramIndexData[index].R = R;                          // Set right motor speed
  ramIndexData[index].D = D;                          // Set movement duration or delay (D is often used for delay or straight distance control)
  ramIndexData[index].SA = SA;                        // Set the maximum speed to be used during PID phase
  ramIndexData[index].TA = TA;                        // Set how long PID line-following should be applied (ms)
  ramIndexData[index].modeSensor = modeSensor;        // Set the sensor mode (logical behavior with sensor0 and sensor1)
  ramIndexData[index].action = action;                // Set what action to perform (MOVE, STOP, etc.)
  ramIndexData[index].numPID = numPID;                // Choose which PID constant set to use
  ramIndexData[index].lineColor = lineColor;          // Set the expected color of the line for tracking
}

// Sets the index where the robot should stop executing further path steps.
// Useful in programming specific routines or halting at checkpoints.
void stopAtIndex(byte index) {
  setting.stopIndex = index;  // Store the index where the robot must stop
}

// Sets the PID control parameters for a specific PID configuration index.
// This allows different PID tuning for different path segments.
void setPID(byte num, byte Kp, byte Ki, byte Kd, byte PMax, int PMin) {
  if (num < NUM_PID) {             // Only proceed if num is within allowed PID profiles
    listPID[num].Kp = Kp;          // Proportional gain
    listPID[num].Ki = Ki;          // Integral gain
    listPID[num].Kd = Kd;          // Derivative gain
    listPID[num].PMax = PMax;      // Maximum value the PID output can reach
    listPID[num].PMin = PMin;      // Minimum value (often negative for reverse correction)
  }
}

// Assigns a specific path planning index to a checkpoint.
// Checkpoints allow skipping or starting execution from specific path segments.
void setCheckPoint(byte num, byte index) {
  if (num < NUM_CP) {                  // Only store if checkpoint number is within the defined number of checkpoints
    setting.checkPoint[num] = index;   // Link this checkpoint number to the corresponding behavior index
  }
}



// /**
//  * Pins used for single-pin ultrasonic sonar sensors:
//  * sonarPins[0] => PB12 (Right)
//  * sonarPins[1] => PB13 (Mid)
//  * sonarPins[2] => PA14 (Left)
//  */
// const int sonarPins[NUM_SONARS] = {PB12, PB13, PA14}; 

// // Global sensor readings and flags
// volatile float distances[NUM_SONARS] = {0};           // Stores distance values in cm
// volatile bool readingReady = true;                    // Indicates if a valid echo reading is available
// volatile unsigned long echoStartTime = 0;             // Timestamp for echo signal start

// // Sensor logic control
// int maxDistance = 200;                                // Maximum measurable distance (used as timeout fallback)
// int currentSensor = 0;                                // Index of the current sonar sensor being triggered
// unsigned long maxEchoDuration = 10000;                // Maximum echo wait time in microseconds (~10ms)

// // Interrupt Service Routine: called on rising/falling edge of echo pin
// void echoISR() {
//   if (digitalRead(sonarPins[currentSensor]) == HIGH) {
//     // Echo pulse started
//     echoStartTime = micros();
//   } else {
//     // Echo pulse ended, calculate duration
//     unsigned long echoEndTime = micros();
//     unsigned long duration = echoEndTime - echoStartTime;
//     distances[currentSensor] = (float)duration * 0.0343 / 2.0; // Convert microseconds to centimeters
//     readingReady = true;
//   }
// }


// ///////////////////////////////////////////////////////////////////////////////////////////////////
// // Main sonar logic: triggers one sensor at a time in non-blocking mode
// void updateSonarReading() {
//   // Proceed if previous reading is complete or timeout occurred
//   if (readingReady || (micros() - echoStartTime > maxEchoDuration)) {
    
//     // Timeout fallback: set max distance if no echo was detected
//     if (!readingReady && (micros() - echoStartTime > maxEchoDuration)) {
//       distances[currentSensor] = maxDistance;
//     }

//     // Move to next sensor in a cyclic manner
//     currentSensor = (currentSensor + 1) % NUM_SONARS;

//     // Disable other sonar pins and set them LOW
//     for (int i = 0; i < NUM_SONARS; i++) {
//       if (i != currentSensor) {
//         detachInterrupt(digitalPinToInterrupt(sonarPins[i]));
//         pinMode(sonarPins[i], OUTPUT);
//         digitalWrite(sonarPins[i], LOW);
//       }
//     }

//     // Trigger ultrasonic pulse on the current sonar
//     detachInterrupt(digitalPinToInterrupt(sonarPins[currentSensor]));
//     pinMode(sonarPins[currentSensor], OUTPUT);
//     digitalWrite(sonarPins[currentSensor], LOW);
//     delayMicroseconds(2);
//     digitalWrite(sonarPins[currentSensor], HIGH);
//     delayMicroseconds(10);
//     digitalWrite(sonarPins[currentSensor], LOW);

//     // Switch pin to input to receive echo and reattach interrupt
//     pinMode(sonarPins[currentSensor], INPUT);
//     attachInterrupt(digitalPinToInterrupt(sonarPins[currentSensor]), echoISR, CHANGE);

//     // Reset flag for next echo detection
//     readingReady = false;
//   }
// }


// void sonarInit(){
  
//   // Setup all sonar pins as input initially and attach interrupts
//   for (int i = 0; i < NUM_SONARS; i++) {
//     pinMode(sonarPins[i], INPUT);
//     attachInterrupt(digitalPinToInterrupt(sonarPins[i]), echoISR, CHANGE);
//   }
// }




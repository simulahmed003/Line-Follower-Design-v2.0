#include <Wire.h>                  // I2C communication library
#include "config.h"                // Contains pin definitions, constants, and macros
#include "init.h"                  // Initialization functions (e.g., motors, mux, buzzer)
#include "globals.h"               // Global variables used across files
#include "functions.h"            // Core logic and utility function declarations
#include "path_planing.h"       // Algorithm and checkpoint/index planning
#include <SoftwareSerial.h>       // Library for software-based serial communication
#include "tunnel.h"

// Create a software serial object using PA10 as RX and PA9 as TX
SoftwareSerial mySerial(PA10, PA9); 

// OPTIONAL: OLED display (currently commented out for future use)
// #include <U8g2lib.h>
// U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);

/**
 * @brief Arduino setup function – runs once at startup.
 * Initializes serial communication, hardware modules like mux, motor, buzzer, and buttons.
 */
void setup() {
  // Initialize software serial port at 115200 baud rate
  mySerial.begin(115200);
  mySerial.println("Hello, world?"); // Startup message for debug

  if(previouslyCalibrated){
    for (int i = 0; i < 16; i++) {
      setting.LIMIT_VALUE_SENSOR[i] = preDefinedThreshold[i];
    }
  }

  // Initialize all essential modules
  startMux();       // Starts multiplexer (likely for IR sensors)
  startMotor();     // Initializes motor pins and sets default values
  startBuzzer();    // Sets up buzzer pin (for alerts or feedback)
  startButton();    // Sets up button input pins and configurations
}

/**
 * @brief Main loop function – runs repeatedly after setup().
 * Currently used to test LED, button press detection, and analog readings.
 */
void loop() {
  // Blink onboard blue LED every 500ms for basic status indication
  digitalWrite(ledPinBlue, HIGH);
  delay(5);
  digitalWrite(ledPinBlue, LOW);
  delay(5);

  // ----------- DEBUGGING SENSOR READINGS (commented for later use) ------------
  uint16_t sensorBits = readSensor(); // Read 16-bit binary sensor data

  // Print binary sensor states to SoftwareSerial
  mySerial.print("Sensor Bits: ");
  for (int i = 15; i >= 0; i--) {
    mySerial.print(bitRead(sensorBits, i));
  }
  mySerial.println();

  // // Print raw analog values from 16 sensors
  // mySerial.print("Raw Readings: ");
  // for (int i = 0; i < 16; i++) {
  //   mySerial.print(reading[i]);
  //   mySerial.print(i < 15 ? ", " : "\n");
  // }

  // // ----------- ANALOG BUTTON TESTING SECTION ------------
  // mySerial.print(analogRead(SW0));  // Read analog value of Button SW0
  // mySerial.print("  ");
  // mySerial.println(analogRead(SW1)); // Read analog value of Button SW1

  // Check if either button SW0 or SW1 is pressed
  // if (detectButtonPress(SW0) || detectButtonPress(SW1)) {
  //   // Log press details from currentButtonInfo struct
  //   mySerial.print("Button ");
  //   mySerial.print(currentButtonInfo.id);
  //   mySerial.print(" pressed for ");
  //   mySerial.print(currentButtonInfo.duration);
  //   mySerial.print(" ms, Count: ");
  //   mySerial.println(currentButtonInfo.pressCount);
  // }



  // Arduino main loop

  updateSonarReading(); // Perform non-blocking sonar update

  // Output sensor distances via SoftwareSerial
  mySerial.print("Right: ");
  mySerial.print(distances[0]);
  mySerial.print(" cm, Mid: ");
  mySerial.print(distances[1]);
  mySerial.print(" cm, Left: ");
  mySerial.println(distances[2]);





  
}

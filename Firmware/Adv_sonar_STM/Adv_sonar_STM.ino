#include <SoftwareSerial.h>

// Create a software serial object using PA10 as RX and PA9 as TX
SoftwareSerial mySerial(PA10, PA9);

// ---------------------- Sonar Sensor Pins ----------------------
#define NUM_SONARS 3
const int sonarPins[NUM_SONARS] = {PB12, PB13, PA14}; // Right, Mid, Left

volatile float distances[NUM_SONARS] = {0};  // Distance readings in cm
volatile bool readingReady = true;
volatile unsigned long echoStartTime = 0;

int maxDistance =200;

int currentSensor = 0;
unsigned long maxEchoDuration = 10000; // microseconds ~110000 to 200 cm

// Interrupt Service Routine for echo signal
void echoISR() {
  if (digitalRead(sonarPins[currentSensor]) == HIGH) {
    echoStartTime = micros();  // Echo pulse started
  } else {
    unsigned long echoEndTime = micros();
    unsigned long duration = echoEndTime - echoStartTime;

    distances[currentSensor] = (float)duration * 0.0343 / 2.0; // convert to cm
    readingReady = true;
  }
}

// Trigger and listen on one sonar pin at a time
void updateSonarReading() {


  if (readingReady || (micros() - echoStartTime > maxEchoDuration)) {

    if((micros() - echoStartTime > maxEchoDuration) & !readingReady){
      distances[currentSensor] = maxDistance;
    }
    // Move to next sensor cyclically
    currentSensor = (currentSensor + 1) % NUM_SONARS;

    // Turn off other sonar pins
    for (int i = 0; i < NUM_SONARS; i++) {
      if (i != currentSensor) {
        detachInterrupt(digitalPinToInterrupt(sonarPins[i]));
        pinMode(sonarPins[i], OUTPUT);
        digitalWrite(sonarPins[i], LOW);
      }
    }

    // Trigger pulse on current sonar
    detachInterrupt(digitalPinToInterrupt(sonarPins[currentSensor]));
    pinMode(sonarPins[currentSensor], OUTPUT);
    digitalWrite(sonarPins[currentSensor], LOW);
    delayMicroseconds(2);
    digitalWrite(sonarPins[currentSensor], HIGH);
    delayMicroseconds(10);
    digitalWrite(sonarPins[currentSensor], LOW);

    // Switch to input mode and listen
    pinMode(sonarPins[currentSensor], INPUT);
    attachInterrupt(digitalPinToInterrupt(sonarPins[currentSensor]), echoISR, CHANGE);
    readingReady = false;

  }
}

void setup() {
  mySerial.begin(115200);

  // Initialize sonar pins as input with interrupts
  for (int i = 0; i < NUM_SONARS; i++) {
    pinMode(sonarPins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(sonarPins[i]), echoISR, CHANGE);
  }
}

void loop() {
  updateSonarReading();

  // Print all distances via SoftwareSerial
  mySerial.print("Right: ");
  mySerial.print(distances[0]);
  mySerial.print(" cm, Mid: ");
  mySerial.print(distances[1]);
  mySerial.print(" cm, Left: ");
  mySerial.println(distances[2]);

  delay(100);  // Optional delay
}

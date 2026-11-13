

#include <SoftwareSerial.h>       // Library for software-based serial communication

// Create a software serial object using PA10 as RX and PA9 as TX
SoftwareSerial mySerial(PA10, PA9); 

const int trigEchoPin = PB12;  // Change according to your STM32 pin

volatile bool readingReady = false;
volatile float distance_cm = 0.0;
volatile unsigned long echoStartTime = 0;

void echo_isr() {
  if (digitalRead(trigEchoPin) == HIGH) {
    // Rising edge: echo pulse started
    echoStartTime = micros();
  } else {
    // Falling edge: echo pulse ended
    unsigned long echoEndTime = micros();
    unsigned long duration = echoEndTime - echoStartTime;
    distance_cm = duration * 0.0343 / 2.0;
    readingReady = true;
  }
}

void setup() {
    // Initialize software serial port at 115200 baud rate
  mySerial.begin(115200);
  mySerial.println("Hello, world?"); // Startup message for debug

  pinMode(trigEchoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(trigEchoPin), echo_isr, CHANGE);
}
 
void loop() {
  // Trigger sonar pulse
  detachInterrupt(digitalPinToInterrupt(trigEchoPin));
  pinMode(trigEchoPin, OUTPUT);
  digitalWrite(trigEchoPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigEchoPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigEchoPin, LOW);

  // Listen for echo
  pinMode(trigEchoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(trigEchoPin), echo_isr, CHANGE);
  readingReady = false;

  delayMicroseconds(500);  // Allow echo to complete

  mySerial.println(distance_cm);

  delay(100);  // Adjust as needed
}


#include <Arduino.h>
#include "config.h"
#include "init.h"
#include "path_planing.h"

/**
 * @brief Initializes the multiplexer (MUX) control and signal pins.
 *        MUX is used to select between multiple analog line sensors.
 */
void startMux() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(sig, INPUT_ANALOG);  // Analog signal input from selected sensor

  // Set all selector lines to 0 (default to sensor 0)
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
}

/**
 * @brief Initializes motor control pins.
 *        Sets PWM and direction pins as outputs and ensures motors are stopped initially.
 */
void startMotor() {
  pinMode(pwmR, OUTPUT);  // Right motor PWM
  pinMode(pwmL, OUTPUT);  // Left motor PWM
  pinMode(dirR, OUTPUT);  // Right motor direction
  pinMode(dirL, OUTPUT);  // Left motor direction

  // Stop motors on startup
  digitalWrite(pwmL, LOW);
  digitalWrite(pwmR, LOW);
  digitalWrite(dirL, LOW);
  digitalWrite(dirR, LOW);
}

/**
 * @brief Initializes the buzzer and status LEDs.
 *        Used for audio and visual debugging/feedback.
 */
void startBuzzer() {
  pinMode(buzzerPin, OUTPUT);    // Buzzer for beeps/alerts
  pinMode(ledPinRed, OUTPUT);    // Red LED (usually error or warning)
  pinMode(ledPinBlue, OUTPUT);   // Blue LED (usually status or active indicator)
}

/**
 * @brief Initializes user input buttons.
 *        These are used to trigger start/stop/reset or other manual actions.
 */
void startButton() {
  pinMode(SW1, INPUT);  // Button SW1
  pinMode(SW0, INPUT);  // Button SW0
}

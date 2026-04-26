#ifndef CONFIG_H
#define CONFIG_H

/**
 * @file config.h
 * @brief Pin assignments for motor driver, sensors, buttons, buzzer, and LED.
 *        All GPIO pin mappings are defined here for easy reconfiguration.
 */

// ---------------------- Motor Control Pins ----------------------
#define pwmR PB0      ///< PWM pin for right motor speed control
#define pwmL PB1      ///< PWM pin for left motor speed control
#define dirR PB2      ///< Direction control for right motor
#define dirL PB10     ///< Direction control for left motor

// ---------------------- Multiplexer (Sensor Selector) ----------------------
#define s0  PA7       ///< Mux select pin S0
#define s1  PA6       ///< Mux select pin S1
#define s2  PA5       ///< Mux select pin S2
#define s3  PA4       ///< Mux select pin S3
#define sig PA3       ///< Mux signal output (shared line from sensors)

// ---------------------- Sonar Sensor Pins ----------------------
#define rightSonar  PB12  ///< Trigger/Echo pin for right ultrasonic sensor
#define midSonar    PB13  ///< Trigger/Echo pin for middle ultrasonic sensor
#define leftSonar   PA14  ///< Trigger/Echo pin for left ultrasonic sensor

// ---------------------- Buzzer and LEDs ----------------------
#define buzzerPin   PC15  ///< Buzzer pin for audio feedback
#define ledPinRed   PC14  ///< Red LED indicator (can be used for errors)
#define ledPinBlue  PC13  ///< Blue LED indicator (can be used for status)

// ---------------------- Button Input Pins ----------------------
#define SW1 PA1           ///< Button SW1 input pin
#define SW0 PA0           ///< Button SW0 input pin

// ---------------------- Optional Encoder Pins (Commented Out) ----------------------
// #define enL_PinA PB8     ///< Left motor encoder Channel A
// #define enL_PinB PB9     ///< Left motor encoder Channel B
// #define enR_PinA PA15    ///< Right motor encoder Channel A
// #define enR_PinB PB3     ///< Right motor encoder Channel B

#endif  // CONFIG_H

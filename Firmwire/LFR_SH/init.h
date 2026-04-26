#ifndef INIT_H
#define INIT_H

/**
 * @file init.h
 * @brief Function declarations for initializing hardware peripherals.
 *
 * These functions configure pins and initial states for:
 * - Multiplexer for sensor selection
 * - Motor drivers
 * - Buzzer and indicator LEDs
 * - User control buttons
 */

/**
 * @brief Initialize multiplexer (MUX) control and signal input pins.
 */
void startMux();

/**
 * @brief Initialize motor PWM and direction control pins.
 */
void startMotor();

/**
 * @brief Initialize buzzer and status LED output pins.
 */
void startBuzzer();

/**
 * @brief Initialize input pins for user buttons.
 */
void startButton();

#endif // INIT_H

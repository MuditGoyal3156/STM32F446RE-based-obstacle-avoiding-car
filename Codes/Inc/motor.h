/*
 * motor.h
 *
 *  Created on: Nov 9, 2025
 *      Author: mudit
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

// Pin definitions for L298N motor driver
#define MOTOR_LEFT_IN1_PIN    8   // PB8
#define MOTOR_LEFT_IN2_PIN    9   // PB9
#define MOTOR_RIGHT_IN3_PIN   10  // PB10
#define MOTOR_RIGHT_IN4_PIN   4   // PB4

// Function prototypes
void motor_init(void);
void motor_forward(void);
void motor_turn_left(void);
void motor_turn_right(void);
void motor_stop(void);

#endif /* MOTOR_H_ */

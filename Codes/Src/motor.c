/*
 * motor.c
 *
 *  Created on: Nov 9, 2025
 *      Author: mudit
 */

#include "motor.h"
#include "stm32f4xx.h"

void motor_init(void)
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= (1U << 1);

    // Configure PB8, PB9, PB10, PB4 as output
    GPIOB->MODER &= ~((3U << 16) | (3U << 18) | (3U << 20) | (3U << 8));  // Clear bits
    GPIOB->MODER |= (1U << 16) | (1U << 18) | (1U << 20) | (1U << 8);     // Set as output (01)

    // Set output to push-pull (default)
    GPIOB->OTYPER &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 4));

    // Set to low speed
    GPIOB->OSPEEDR &= ~((3U << 16) | (3U << 18) | (3U << 20) | (3U << 8));

    // No pull-up/pull-down
    GPIOB->PUPDR &= ~((3U << 16) | (3U << 18) | (3U << 20) | (3U << 8));

    // Initialize motors as stopped
    motor_stop();
}

// Move forward - both motors forward
void motor_forward(void)
{
    // Left motor forward: IN1 = HIGH, IN2 = LOW
    GPIOB->BSRR = (1U << MOTOR_LEFT_IN1_PIN);           // Set PB8 HIGH
    GPIOB->BSRR = (1U << (MOTOR_LEFT_IN2_PIN + 16));    // Set PB9 LOW

    // Right motor forward: IN3 = HIGH, IN4 = LOW
    GPIOB->BSRR = (1U << MOTOR_RIGHT_IN3_PIN);          // Set PB10 HIGH
    GPIOB->BSRR = (1U << (MOTOR_RIGHT_IN4_PIN + 16));   // Set PB4 LOW
}

// Turn left - left motor backward, right motor forward
void motor_turn_left(void)
{
    // Left motor backward: IN1 = LOW, IN2 = HIGH
    GPIOB->BSRR = (1U << (MOTOR_LEFT_IN1_PIN + 16));    // Set PB8 LOW
    GPIOB->BSRR = (1U << MOTOR_LEFT_IN2_PIN);           // Set PB9 HIGH

    // Right motor forward: IN3 = HIGH, IN4 = LOW
    GPIOB->BSRR = (1U << MOTOR_RIGHT_IN3_PIN);          // Set PB10 HIGH
    GPIOB->BSRR = (1U << (MOTOR_RIGHT_IN4_PIN + 16));   // Set PB4 LOW
}

// Turn right - left motor forward, right motor backward
void motor_turn_right(void)
{
    // Left motor forward: IN1 = HIGH, IN2 = LOW
    GPIOB->BSRR = (1U << MOTOR_LEFT_IN1_PIN);           // Set PB8 HIGH
    GPIOB->BSRR = (1U << (MOTOR_LEFT_IN2_PIN + 16));    // Set PB9 LOW

    // Right motor backward: IN3 = LOW, IN4 = HIGH
    GPIOB->BSRR = (1U << (MOTOR_RIGHT_IN3_PIN + 16));   // Set PB10 LOW
    GPIOB->BSRR = (1U << MOTOR_RIGHT_IN4_PIN);          // Set PB4 HIGH
}

// Stop both motors
void motor_stop(void)
{
    // All pins LOW to stop both motors
    GPIOB->BSRR = (1U << (MOTOR_LEFT_IN1_PIN + 16));     // Set PB8 LOW
    GPIOB->BSRR = (1U << (MOTOR_LEFT_IN2_PIN + 16));     // Set PB9 LOW
    GPIOB->BSRR = (1U << (MOTOR_RIGHT_IN3_PIN + 16));    // Set PB10 LOW
    GPIOB->BSRR = (1U << (MOTOR_RIGHT_IN4_PIN + 16));    // Set PB4 LOW
}

# **STM32F446RE-Based Obstacle Avoiding Car**

An obstacle avoiding robotic vehicle built using the STM32F446RE microcontroller and an HC-SR04 ultrasonic sensor.
The project is implemented entirely in bare-metal C, with all peripherals configured using direct register-level programming (no HAL / no LL).

## **Project Overview**

This project demonstrates the design and implementation of an embedded obstacle avoidance system using an STM32 microcontroller.
The vehicle continuously measures the distance to obstacles using an ultrasonic sensor and reacts in real time by:

Providing visual and auditory feedback

Controlling motors to avoid collisions

Streaming sensor data over UART for debugging and monitoring

The focus of the project is on low-level embedded system design, emphasizing timing accuracy, peripheral configuration, and modular driver development.

## **System Description**

The ultrasonic sensor periodically emits a trigger pulse and measures the duration of the returned echo signal. This time-of-flight measurement is converted into distance.

Based on the measured distance, the system performs the following actions:

### **Obstacle Detection**

Continuously monitors the distance ahead of the vehicle

Classifies proximity into safe, warning, and danger zones

### **Motor Control**

Moves forward when the path is clear

Slows down, stops, or changes direction when an obstacle is detected

### **User Feedback**

LED indicators show proximity levels

Buzzer provides audible alerts with increasing frequency as distance decreases

UART output sends distance data to a serial terminal

## **Features**

Real-Time Distance Measurement using ultrasonic sensing

Bare-Metal Peripheral Control using register-level programming

Motor Control Logic for obstacle avoidance

Visual Feedback using LEDs

Audi tory Feedback using a buzzer

UART-Based Debugging Output

Modular Driver Architecture

## **Hardware Components**

STM32F446RE Nucleo Development Board

HC-SR04 Ultrasonic Distance Sensor

DC Motors with Motor Driver Module

LEDs (Green, Yellow, Red)

Piezo Buzzer

Power Supply (Battery / USB)

Breadboard and jumper wires

## **Software Environment**

Programming Language: C (Bare-Metal)

Toolchain: ARM GCC (arm-none-eabi-gcc)

IDE: STM32CubeIDE / Keil

Debug Interface: ST-Link

Serial Terminal: PuTTY / Tera Term / RealTerm

## **Functional Modules**

The project is organized into modular source files, each responsible for a specific function:

**Ultrasonic Sensor Module**
Handles trigger generation, echo timing, and distance calculation.

**Timing Module**
Provides precise delays and time measurements required for ultrasonic sensing and control logic.

**Motor Control Module**
Controls direction and movement of the vehicle based on obstacle distance.

**Buzzer Module**
Generates audible alerts with variable frequency depending on proximity.

**UART Module**
Enables serial communication for debugging and monitoring.

**Interrupt Module**
Manages interrupt configuration and handling for responsive system behavior.

## **Working Principle**

The microcontroller triggers the ultrasonic sensor.

Echo pulse duration is measured and converted to distance.

Distance thresholds determine vehicle behavior.

Motors respond to avoid obstacles.

LEDs and buzzer provide real-time feedback.

Distance data is transmitted via UART.

## **Future Enhancements**

EXTI-based echo measurement for non-blocking operation

Advanced motor control with PID

Sensor fusion using IR or ToF sensors

LCD/OLED display for standalone operation

Wireless telemetry (Bluetooth / Wi-Fi)

## **Conclusion**

This project successfully demonstrates a real-time obstacle avoidance system using the STM32F446RE microcontroller.
By implementing all functionality using bare-metal programming, the project provides a strong foundation for understanding embedded system internals and real-world hardware interaction.

#include "stm32f4xx.h"
#include "hcsr04.h"
#include "uart.h"
#include "timing.h"
#include "buzz.h"
#include "int.h"
#include "motor.h"
#include <stdio.h>

// Define the duration of each beep in milliseconds
#define BEEP_DURATION_MS 50
#define STOP_DISTANCE_CM 10  // Stop motor when distance < 10cm (1000 in cm*100)
#define TURN_DURATION_MS 800 // Turn for 800ms
#define SCAN_DELAY_MS 300    // Delay after turning to stabilize before measuring

// Global flag to control system state
volatile uint8_t system_active = 0;  // 0 = stopped, 1 = running

// Function to scan and decide direction
void scan_and_decide_direction(void)
{
    int32_t left_distance = 0;
    int32_t right_distance = 0;
    int32_t front_distance = 0;

    printf("Obstacle detected! Starting scan...\r\n");

    // Stop the motor
    motor_stop();
      // Stop for 500ms

    // Turn left and measure
    printf("Scanning left...\r\n");
    motor_turn_left();
    delay_us(TURN_DURATION_MS * 1000); // Turn left for 800ms
    motor_stop();
    delay_us(SCAN_DELAY_MS * 1000);    // Wait for stabilization

    left_distance = HCSR04_ReadDistance_cm_x100(30);
    if(left_distance > 0) {
        printf("Left distance: %d.%02d cm\r\n", left_distance / 100, left_distance % 100);
    } else {
        printf("Left distance: Out of range (assuming clear)\r\n");
        left_distance = 10000; // Assume clear path if out of range
    }

    // Turn back to center (turn right)
    printf("Returning to center...\r\n");
    motor_turn_right();
    delay_us(TURN_DURATION_MS * 1000); // Turn right for 800ms to center
    motor_stop();
    delay_us(SCAN_DELAY_MS * 1000);

    front_distance = HCSR04_ReadDistance_cm_x100(30);
    if(front_distance > 0) {
        printf("Front distance: %d.%02d cm\r\n", front_distance / 100, front_distance % 100);
    } else {
        printf("Front distance: Out of range (assuming clear)\r\n");
        front_distance = 10000;
    }

    // Turn right and measure
    printf("Scanning right...\r\n");
    motor_turn_right();
    delay_us(TURN_DURATION_MS * 1000); // Turn right for 800ms
    motor_stop();
    delay_us(SCAN_DELAY_MS * 1000);

    right_distance = HCSR04_ReadDistance_cm_x100(30);
    if(right_distance > 0) {
        printf("Right distance: %d.%02d cm\r\n", right_distance / 100, right_distance % 100);
    } else {
        printf("Right distance: Out of range (assuming clear)\r\n");
        right_distance = 10000;
    }

    // Turn back to center (turn left)
    printf("Returning to center...\r\n");
    motor_turn_left();
    delay_us(TURN_DURATION_MS * 1000); // Turn left for 800ms to center
    motor_stop();
    delay_us(SCAN_DELAY_MS * 1000);

    // Decide which direction has more space
    printf("\r\n=== Scan Results ===\r\n");
    printf("Left: %d.%02d cm | Front: %d.%02d cm | Right: %d.%02d cm\r\n",
           left_distance / 100, left_distance % 100,
           front_distance / 100, front_distance % 100,
           right_distance / 100, right_distance % 100);

    // Find the direction with maximum space
    if (left_distance >= right_distance && left_distance >= front_distance) {
        printf("Decision: Turn LEFT (most space)\r\n");
        motor_turn_left();
        delay_us(TURN_DURATION_MS * 1000);
        motor_stop();
    }
    else if (right_distance >= left_distance && right_distance >= front_distance) {
        printf("Decision: Turn RIGHT (most space)\r\n");
        motor_turn_right();
        delay_us(TURN_DURATION_MS * 1000);
        motor_stop();
    }
    else {
        printf("Decision: Continue FORWARD (most space)\r\n");
        // Already centered, just continue
    }

    printf("Scan complete. Resuming forward movement...\r\n\r\n");
}

int main(void)
{
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2)); //FPU enable

    // Initialize UART first and add delay
    uart2_tx_init();

    // Small delay to let UART stabilize
    for(volatile uint32_t i = 0; i < 100000; i++);

    DWT_Delay_Init();
    HCSR04_Init();
    buzzer_init();
    GPIO();
    INT();
    motor_init();

    // LED GPIO init (PA5, PA6, PA7)
    RCC->AHB1ENR |= (1U << 0);
    GPIOA->MODER |= (1U << 10) | (1U << 12) | (1U << 14); // Set PA5, PA6, PA7 as output

    int32_t distance;

    // Add delay before first printf
    delay_us(100000); // 100ms delay
    printf("System initialized. Press PC13 to start/stop.\r\n");

    while(1)
    {
        // ALWAYS read and print distance regardless of system state
        distance = HCSR04_ReadDistance_cm_x100(30);

        if(distance > 0)
        {
            printf("Distance: %d.%02d cm\r\n", distance / 100, distance % 100);
        }
        else
        {
            printf("Distance: Out of range\r\n");
        }

        // Only activate buzzer, LEDs, and motor when system is active
        if (system_active)
        {
            // Motor control based on distance with scanning logic
            if (distance > 0 && distance < 1000)  // Less than 10cm
            {
                // Call the scanning function
                scan_and_decide_direction();
            }
            else if (distance > 0)
            {
                motor_forward();
            }

            // LED control based on distance
            if(distance > 0 && distance < 1000)
            {
                GPIOA->ODR = (1 << 5); // Red ON, others OFF
            }
            else if(distance >= 1000 && distance < 3000)
            {
                GPIOA->ODR = (1 << 6); // Yellow ON, others OFF
            }
            else if(distance >= 3000)
            {
                GPIOA->ODR = (1 << 7); // Green ON, others OFF
            }
            else
            {
                GPIOA->ODR &= ~((1 << 5) | (1 << 6) | (1 << 7)); // All OFF
            }

            // Buzzer beeping pattern based on distance
            if (distance > 0 && distance <= 3000)
            {
                // Map distance (1-3000) to a beep interval
                const uint32_t INTERVAL_MAX_MS = 500; // Delay at 30cm
                const uint32_t INTERVAL_MIN_MS = 65;  // Delay at <1cm
                const uint32_t D_MAX_VAL = 3000;

                // Calculate the delay between the start of one beep and the start of the next
                uint32_t beep_interval_ms = INTERVAL_MIN_MS + (((uint64_t)distance * (INTERVAL_MAX_MS - INTERVAL_MIN_MS)) / D_MAX_VAL);

                // Turn the beep on
                buzzer_on();
                delay_us(BEEP_DURATION_MS * 1000); // Beep for a fixed duration

                // Turn the beep off
                buzzer_off();

                // Wait for the rest of the interval before the next beep
                if (beep_interval_ms > BEEP_DURATION_MS)
                {
                    delay_us((beep_interval_ms - BEEP_DURATION_MS) * 1000);
                }
            }
            else
            {
                // If out of range, just wait a bit
                delay_us(50000); // 50ms delay
            }
        }
        else
        {
            // System inactive - turn everything off but keep measuring distance
            buzzer_off();
            motor_stop();
            GPIOA->ODR &= ~((1 << 5) | (1 << 6) | (1 << 7)); // All LEDs OFF
            delay_us(100000); // 100ms delay between distance readings
        }
    }
}

// PC13 interrupt handler - toggles system on/off
void EXTI15_10_IRQHandler(void)
{
    // Check if EXTI13 triggered the interrupt
    if (EXTI->PR & (1U << 13))
    {
        // Clear the pending bit
        EXTI->PR |= (1U << 13);

        // Toggle system state
        system_active = !system_active;

        if (system_active)
        {
            printf("\r\nSystem ACTIVATED\r\n");
        }
        else
        {
            printf("\r\nSystem DEACTIVATED - Distance monitoring only\r\n");
            // Immediately turn everything off
            buzzer_off();
            motor_stop();
            GPIOA->ODR &= ~((1 << 5) | (1 << 6) | (1 << 7));
        }

        // Simple debounce delay
        delay_us(200000); // 200ms debounce
    }
}

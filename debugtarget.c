/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeRTOS version for Raspberry Pi Pico 2 (RP2350)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ARM Cortex-M includes for interrupt priority configuration */
#include "hardware/structs/scb.h"

#define GPIO_WATCH_PIN 14
#define GPIO_TOGGLE_PIN 15
#define GPIO_BUTTON_PIN 16  // External button for breakpoint trigger (connect to GND when pressed)

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

static char event_str[128];

/* Semaphore to signal interrupt event to the interrupt handler task */
static SemaphoreHandle_t xInterruptSemaphore = NULL;

/* Variables to store interrupt information */
static volatile uint32_t last_gpio = 0;
static volatile uint32_t last_events = 0;

/* Repeating timer for LED blink */
static struct repeating_timer led_timer;

void gpio_event_string(char *buf, uint32_t events);

/* GPIO interrupt callback - called from ISR context */
void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Store interrupt information
    last_gpio = gpio;
    last_events = events;

    // Give the semaphore to wake up the interrupt handler task
    xSemaphoreGiveFromISR(xInterruptSemaphore, &xHigherPriorityTaskWoken);

    // Request context switch if a higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Task 1: Toggle GPIO_TOGGLE_PIN every one second */
void vToggleTask(void *pvParameters) {
    printf("[ToggleTask] Started - will toggle GPIO %d every 1 second\n", GPIO_TOGGLE_PIN);

    while (1) {
        gpio_put(GPIO_TOGGLE_PIN, !gpio_get(GPIO_TOGGLE_PIN));
        printf("[ToggleTask] GPIO %d toggled to %d, GPIO %d reads: %d\n",
               GPIO_TOGGLE_PIN, gpio_get(GPIO_TOGGLE_PIN),
               GPIO_WATCH_PIN, gpio_get(GPIO_WATCH_PIN));
        
        // __asm volatile("bkpt #0");

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Task 2: Handle GPIO interrupts - runs idle until interrupt occurs */
void vInterruptHandlerTask(void *pvParameters) {
    printf("[InterruptHandlerTask] Started - waiting for interrupts on GPIO %d\n", GPIO_WATCH_PIN);

    while (1) {
        // Wait indefinitely for the interrupt semaphore
        if (xSemaphoreTake(xInterruptSemaphore, portMAX_DELAY) == pdTRUE) {
            // Process the interrupt event
            gpio_event_string(event_str, last_events);
            printf("[InterruptHandlerTask] GPIO %d interrupt: %s\n", last_gpio, event_str);
        }
    }
}

/* Hardware timer interrupt handler - toggles LED every 500ms */
static bool timer_callback(struct repeating_timer *t) {
    // gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
    return true; // Keep repeating
}

/* Task 3: High priority task that polls button and triggers breakpoint */
void vLedBlinkTask(void *pvParameters) {
    printf("[LedBlinkTask] Started - polling GPIO %d button for breakpoint trigger\n", GPIO_BUTTON_PIN);

    bool last_button_state = true;  // Start high due to pull-up

    while (1) {
        // // Poll the button (active low - reads 0 when pressed)
        // bool current_button_state = gpio_get(GPIO_BUTTON_PIN);

        // // Detect falling edge (button press - transition from high to low)
        // if (!current_button_state && last_button_state) {
        //     printf("[LedBlinkTask] Button on GPIO %d pressed - triggering breakpoint!\n", GPIO_BUTTON_PIN);

        //     // Trigger software breakpoint
        //     __asm volatile("bkpt #0");

        //     printf("[LedBlinkTask] Resumed from breakpoint\n");
        // }

        // last_button_state = current_button_state;

        // __asm volatile("bkpt #0");
        
        // Yield briefly to avoid starving lower-priority tasks
        gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();
    // sleep_ms(2000);  // Wait for USB serial to be ready

    printf("===========================================\n");
    printf("Hello GPIO IRQ with FreeRTOS on RP2350\n");
    printf("===========================================\n");
    printf("Connect GPIO %d to GPIO %d with a jumper wire\n", GPIO_TOGGLE_PIN, GPIO_WATCH_PIN);
    printf("===========================================\n\n");

    // // Wait until Debug Exception and Monitor Control Register (DEMCR).MON_REQ becomes 1
    // // DEMCR address: 0xE000EDFC, MON_REQ bit: 19
    // volatile uint32_t *DEMCR = (volatile uint32_t *)0xE000EDFCu;
    // printf("[Setup] Waiting for DEMCR.MON_REQ (bit 19) to become 1...\n");
    // while (((*DEMCR) & (1u << 19)) == 0) {
    //     tight_loop_contents();
    // }
    // printf("[Setup] DEMCR.MON_REQ is now 1\n");

    // // Clear DEMCR.MON_REQ by writing 1 to bit 19
    // *DEMCR = *DEMCR & ~(1u << 19);
    // printf("[Setup] DEMCR.MON_REQ cleared\n");

    // Setup GPIO 2 as input to watch for interrupts
    gpio_init(GPIO_WATCH_PIN);
    gpio_set_dir(GPIO_WATCH_PIN, GPIO_IN);
    gpio_pull_down(GPIO_WATCH_PIN);  // Enable pull-down resistor
    printf("[Setup] GPIO %d configured as input with pull-down\n", GPIO_WATCH_PIN);

    // Setup GPIO 3 as output to toggle every second
    gpio_init(GPIO_TOGGLE_PIN);
    gpio_set_dir(GPIO_TOGGLE_PIN, GPIO_OUT);
    gpio_put(GPIO_TOGGLE_PIN, 0);
    printf("[Setup] GPIO %d configured as output\n", GPIO_TOGGLE_PIN);

    // Setup LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    printf("[Setup] LED on GPIO %d configured\n", PICO_DEFAULT_LED_PIN);

    // Setup button for breakpoint trigger
    gpio_init(GPIO_BUTTON_PIN);
    gpio_set_dir(GPIO_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(GPIO_BUTTON_PIN);  // Enable pull-up resistor (button connects to GND)
    printf("[Setup] Button on GPIO %d configured (active low, connect to GND to trigger)\n", GPIO_BUTTON_PIN);

    // Configure interrupt priorities BEFORE setting up the timer
    // On Cortex-M33, lower numbers = higher priority
    // Debug Monitor exception priority = 0x80 (128 - lower priority)
    // Timer interrupt priority = 0x40 (64 - higher priority, will preempt debug monitor)

    // Set Debug Monitor exception priority (exception 12)
    // SHPR[2] contains priorities for exception 12 (DebugMonitor)
    scb_hw->shpr[2] = 0x80;  // DebugMonitor priority = 0x80 (128)
    printf("[Setup] Debug Monitor priority set to 0x80 (128)\n");

    // Setup hardware timer to toggle LED every 500ms
    if (!add_repeating_timer_ms(500, timer_callback, NULL, &led_timer)) {
        printf("[Setup] ERROR: Failed to add repeating timer!\n");
        while (1) { tight_loop_contents(); }
    }

    // The repeating timer uses one of TIMER0_IRQ_0-3 or TIMER1_IRQ_0-3
    // Set all timer IRQs to higher priority than debug monitor
    // RP2350 has two timer peripherals (TIMER0 and TIMER1)
    irq_set_priority(TIMER0_IRQ_0, 0x40);
    irq_set_priority(TIMER0_IRQ_1, 0x40);
    irq_set_priority(TIMER0_IRQ_2, 0x40);
    irq_set_priority(TIMER0_IRQ_3, 0x40);
    irq_set_priority(TIMER1_IRQ_0, 0x40);
    irq_set_priority(TIMER1_IRQ_1, 0x40);
    irq_set_priority(TIMER1_IRQ_2, 0x40);
    irq_set_priority(TIMER1_IRQ_3, 0x40);
    printf("[Setup] Timer interrupt priority set to 0x40 (64) - higher than Debug Monitor\n");
    printf("[Setup] Hardware timer configured to toggle LED every 500ms\n");

    // Create binary semaphore for interrupt signaling
    xInterruptSemaphore = xSemaphoreCreateBinary();
    if (xInterruptSemaphore == NULL) {
        printf("[Setup] ERROR: Failed to create semaphore!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("[Setup] Interrupt semaphore created\n");

    // Enable GPIO interrupt with callback
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &gpio_callback);
    printf("[Setup] IRQ enabled on GPIO %d for RISE and FALL edges\n\n", GPIO_WATCH_PIN);

    // Create Task 1: Toggle GPIO every 1 second
    TaskHandle_t xToggleTaskHandle = NULL;
    BaseType_t xReturned = xTaskCreate(
        vToggleTask,           /* Task function */
        "ToggleTask",          /* Task name */
        256,                   /* Stack size (words) */
        NULL,                  /* Task parameters */
        1,                     /* Priority */
        &xToggleTaskHandle     /* Task handle */
    );

    if (xReturned != pdPASS) {
        printf("[Setup] ERROR: Failed to create ToggleTask!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("[Setup] ToggleTask created\n");

    // Create Task 2: Interrupt handler task
    TaskHandle_t xInterruptTaskHandle = NULL;
    xReturned = xTaskCreate(
        vInterruptHandlerTask, /* Task function */
        "InterruptTask",       /* Task name */
        256,                   /* Stack size (words) */
        NULL,                  /* Task parameters */
        2,                     /* Higher priority than toggle task */
        &xInterruptTaskHandle  /* Task handle */
    );

    if (xReturned != pdPASS) {
        printf("[Setup] ERROR: Failed to create InterruptTask!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("[Setup] InterruptTask created\n");

    // Create Task 3: LED blink task (highest priority)
    TaskHandle_t xLedBlinkTaskHandle = NULL;
    xReturned = xTaskCreate(
        vLedBlinkTask,         /* Task function */
        "LedBlinkTask",        /* Task name */
        256,                   /* Stack size (words) */
        NULL,                  /* Task parameters */
        3,                     /* Highest priority */
        &xLedBlinkTaskHandle   /* Task handle */
    );

    if (xReturned != pdPASS) {
        printf("[Setup] ERROR: Failed to create LedBlinkTask!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("[Setup] LedBlinkTask created\n\n");

    printf("Starting FreeRTOS scheduler...\n\n");

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    printf("[Setup] ERROR: Scheduler failed to start!\n");
    while (1) {
        tight_loop_contents();
    }
}


static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

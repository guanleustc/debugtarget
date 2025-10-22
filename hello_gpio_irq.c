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

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define GPIO_WATCH_PIN 2
#define GPIO_TOGGLE_PIN 3

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

static char event_str[128];

/* Semaphore to signal interrupt event to the interrupt handler task */
static SemaphoreHandle_t xInterruptSemaphore = NULL;

/* Variables to store interrupt information */
static volatile uint32_t last_gpio = 0;
static volatile uint32_t last_events = 0;

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
        gpio_put(PICO_DEFAULT_LED_PIN, gpio_get(GPIO_TOGGLE_PIN));
        printf("[ToggleTask] GPIO %d toggled to %d, GPIO %d reads: %d\n",
               GPIO_TOGGLE_PIN, gpio_get(GPIO_TOGGLE_PIN),
               GPIO_WATCH_PIN, gpio_get(GPIO_WATCH_PIN));

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

int main() {
    stdio_init_all();
    // sleep_ms(2000);  // Wait for USB serial to be ready

    printf("===========================================\n");
    printf("Hello GPIO IRQ with FreeRTOS on RP2350\n");
    printf("===========================================\n");
    printf("Connect GPIO 3 to GPIO 2 with a jumper wire\n");
    printf("===========================================\n\n");

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
    printf("[Setup] InterruptTask created\n\n");

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

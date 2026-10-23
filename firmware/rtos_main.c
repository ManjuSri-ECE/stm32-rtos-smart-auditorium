/*
 * rtos_main.c
 * STM32F407VGT6 — FreeRTOS starter implementation using CMSIS register access
 *
 * STATUS:
 * - Separate example implementation; not original/recovered project firmware.
 * - Proposed pin map, not verified against the original prototype.
 * - Not hardware-tested. Requires CMSIS device headers, startup/linker files,
 *   FreeRTOS kernel and port, and an appropriate STM32F407 project/toolchain.
 *
 * Proposed wiring:
 *   HC-SR04 A: TRIG PA0, ECHO PA1
 *   HC-SR04 B: TRIG PA2, ECHO PA3
 *   I2C1 LCD:   SCL PB8, SDA PB9
 *
 * Electrical safety:
 *   HC-SR04 ECHO commonly outputs 5 V. Level-shift/divide to 3.3 V.
 *   Ensure LCD I2C pull-ups do not pull SDA/SCL to 5 V.
 *
 * This file shows an RTOS task architecture and peripheral access style.
 * The I2C LCD driver and ultrasonic timing below are deliberately kept as
 * integration hooks rather than pretending to be a validated board driver.
 * Complete and test them for your exact hardware before claiming operation.
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>
#include <stdbool.h>

#define TRIG_A_PIN 0U
#define ECHO_A_PIN 1U
#define TRIG_B_PIN 2U
#define ECHO_B_PIN 3U
#define DETECT_CM  80U
#define SEQUENCE_MS 1500U

typedef struct {
    bool sensor_a;
    bool sensor_b;
    TickType_t sampled_at;
} sensor_event_t;

typedef struct {
    uint32_t in_count;
    uint32_t out_count;
    uint32_t inside_count;
} occupancy_t;

static QueueHandle_t sensor_queue;
static QueueHandle_t display_queue;

static void gpio_init_register_level(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    /* PA0/PA2 outputs; PA1/PA3 inputs. */
    GPIOA->MODER &= ~((3U << (TRIG_A_PIN * 2U)) |
                      (3U << (ECHO_A_PIN * 2U)) |
                      (3U << (TRIG_B_PIN * 2U)) |
                      (3U << (ECHO_B_PIN * 2U)));
    GPIOA->MODER |= (1U << (TRIG_A_PIN * 2U)) |
                    (1U << (TRIG_B_PIN * 2U));
    GPIOA->OTYPER &= ~((1U << TRIG_A_PIN) | (1U << TRIG_B_PIN));
    GPIOA->BSRR = (1U << (TRIG_A_PIN + 16U)) |
                  (1U << (TRIG_B_PIN + 16U));

    /* PB8/PB9 alternate function 4 (I2C1), open-drain. */
    GPIOB->MODER = (GPIOB->MODER & ~((3U << 16U) | (3U << 18U))) |
                   (2U << 16U) | (2U << 18U);
    GPIOB->OTYPER |= (1U << 8U) | (1U << 9U);
    GPIOB->PUPDR &= ~((3U << 16U) | (3U << 18U));
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFU << 0U) | (0xFU << 4U))) |
                    (4U << 0U) | (4U << 4U);
}

/*
 * TODO: Implement a timer-based HC-SR04 read with bounded waits and suitable
 * microsecond timing. Return true when a valid distance is below threshold.
 * Avoid long blocking waits in the acquisition task; use a hardware timer or
 * input capture for robust timing.
 */
static bool read_sensor_detected(uint32_t trig_pin, uint32_t echo_pin)
{
    (void)trig_pin;
    (void)echo_pin;
    /* Placeholder by design: no fabricated distance readings. */
    return false;
}

/* TODO: Initialize I2C1 and the actual LCD backpack, then send display text. */
static void lcd_driver_init(void)
{
    /* Add verified I2C timing and HD44780 backpack driver here. */
}

/* TODO: Implement a real LCD refresh using the verified LCD driver. */
static void lcd_display_occupancy(const occupancy_t *state)
{
    (void)state;
    /* Display IN, OUT, INSIDE once the LCD driver is implemented. */
}

static void SensorTask(void *argument)
{
    (void)argument;
    const TickType_t period = pdMS_TO_TICKS(50U);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        sensor_event_t event;
        event.sensor_a = read_sensor_detected(TRIG_A_PIN, ECHO_A_PIN);
        event.sensor_b = read_sensor_detected(TRIG_B_PIN, ECHO_B_PIN);
        event.sampled_at = xTaskGetTickCount();

        /* Drop old sample rather than block acquisition indefinitely. */
        (void)xQueueSend(sensor_queue, &event, 0U);
        vTaskDelayUntil(&last_wake, period);
    }
}

static void CountingTask(void *argument)
{
    (void)argument;
    sensor_event_t event;
    occupancy_t state = {0U, 0U, 0U};

    enum { WAITING, A_FIRST, B_FIRST } sequence = WAITING;
    TickType_t sequence_start = 0U;
    TickType_t last_count = 0U;
    const TickType_t sequence_timeout = pdMS_TO_TICKS(SEQUENCE_MS);
    const TickType_t cooldown = pdMS_TO_TICKS(1000U);

    for (;;) {
        if (xQueueReceive(sensor_queue, &event, portMAX_DELAY) == pdPASS) {
            TickType_t now = event.sampled_at;

            if (sequence != WAITING &&
                (now - sequence_start) > sequence_timeout) {
                sequence = WAITING;
            }

            if ((now - last_count) >= cooldown) {
                if (sequence == WAITING) {
                    if (event.sensor_a && !event.sensor_b) {
                        sequence = A_FIRST;
                        sequence_start = now;
                    } else if (event.sensor_b && !event.sensor_a) {
                        sequence = B_FIRST;
                        sequence_start = now;
                    }
                } else if (sequence == A_FIRST && event.sensor_b) {
                    state.in_count++;
                    state.inside_count = state.in_count - state.out_count;
                    sequence = WAITING;
                    last_count = now;
                    (void)xQueueOverwrite(display_queue, &state);
                } else if (sequence == B_FIRST && event.sensor_a) {
                    state.out_count++;
                    if (state.out_count > state.in_count) {
                        state.out_count = state.in_count;
                    }
                    state.inside_count = state.in_count - state.out_count;
                    sequence = WAITING;
                    last_count = now;
                    (void)xQueueOverwrite(display_queue, &state);
                }
            }
        }
    }
}

static void DisplayTask(void *argument)
{
    (void)argument;
    occupancy_t state = {0U, 0U, 0U};

    lcd_driver_init();

    for (;;) {
        if (xQueueReceive(display_queue, &state, pdMS_TO_TICKS(250U)) == pdPASS) {
            lcd_display_occupancy(&state);
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

int main(void)
{
    /*
     * For a production CubeIDE project, add the correct clock setup before
     * enabling timing-dependent peripherals. This example intentionally
     * does not assume a specific board clock tree.
     */
    gpio_init_register_level();

    sensor_queue = xQueueCreate(8U, sizeof(sensor_event_t));
    display_queue = xQueueCreate(1U, sizeof(occupancy_t));
    if ((sensor_queue == NULL) || (display_queue == NULL)) {
        for (;;) { /* Allocation failure: halt safely. */ }
    }

    if (xTaskCreate(SensorTask, "Sensor", 256U, NULL,
                    tskIDLE_PRIORITY + 3U, NULL) != pdPASS) {
        for (;;) {}
    }
    if (xTaskCreate(CountingTask, "Count", 256U, NULL,
                    tskIDLE_PRIORITY + 2U, NULL) != pdPASS) {
        for (;;) {}
    }
    if (xTaskCreate(DisplayTask, "Display", 256U, NULL,
                    tskIDLE_PRIORITY + 1U, NULL) != pdPASS) {
        for (;;) {}
    }

    vTaskStartScheduler();
    for (;;) {}
}

/*
 * baremetal_main.c
 * STM32F407VGT6 — register-level bare-metal starter implementation
 *
 * STATUS:
 * - New example implementation based on the documented project concept.
 * - Proposed pin map, not verified against the original prototype.
 * - Not hardware-tested. Review clock setup, board pin availability, LCD backpack
 *   voltage/pull-ups, and sensor level shifting before connecting hardware.
 *
 * Proposed wiring:
 *   HC-SR04 A: TRIG PA0, ECHO PA1
 *   HC-SR04 B: TRIG PA2, ECHO PA3
 *   I2C1 LCD:   SCL PB8, SDA PB9
 *
 * Electrical safety:
 *   HC-SR04 ECHO is commonly 5 V; use a divider/level shifter to protect the
 *   STM32's 3.3 V GPIO. Ensure LCD I2C pull-ups do not pull SDA/SCL to 5 V.
 *
 * This uses CMSIS device definitions (stm32f4xx.h), not STM32 HAL.
 * Startup code, linker script, CMSIS device headers, and system startup files
 * must be supplied by an STM32F407 project/toolchain.
 */

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

#define TRIG_A_PIN     0U
#define ECHO_A_PIN     1U
#define TRIG_B_PIN     2U
#define ECHO_B_PIN     3U

#define LCD_I2C_ADDR   (0x27U) /* Common backpack address; verify yours. */
#define LCD_BACKLIGHT  0x08U
#define LCD_ENABLE     0x04U
#define LCD_RS         0x01U

#define DETECT_CM      80U
#define COOLDOWN_MS    1000U
#define SEQUENCE_MS    1500U
#define LOOP_PERIOD_MS 10U

static volatile uint32_t g_ms_ticks;
static uint32_t g_in_count;
static uint32_t g_out_count;
static uint32_t g_inside_count;

typedef enum {
    SEQ_IDLE = 0,
    SEQ_A_FIRST,
    SEQ_B_FIRST
} sequence_state_t;

static sequence_state_t g_sequence = SEQ_IDLE;
static uint32_t g_sequence_start_ms;
static uint32_t g_last_count_ms;

/* ---------- Basic timebase: SysTick at 1 ms ---------- */
void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {
        __NOP();
    }
}

/* ---------- GPIO helpers ---------- */
static void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    port->MODER = (port->MODER & ~(3U << (pin * 2U))) | (mode << (pin * 2U));
}

static void gpio_output_type_push_pull(GPIO_TypeDef *port, uint32_t pin)
{
    port->OTYPER &= ~(1U << pin);
}

static void gpio_speed(GPIO_TypeDef *port, uint32_t pin, uint32_t speed)
{
    port->OSPEEDR = (port->OSPEEDR & ~(3U << (pin * 2U))) | (speed << (pin * 2U));
}

static void gpio_pull(GPIO_TypeDef *port, uint32_t pin, uint32_t pull)
{
    port->PUPDR = (port->PUPDR & ~(3U << (pin * 2U))) | (pull << (pin * 2U));
}

static void gpio_write(GPIO_TypeDef *port, uint32_t pin, bool high)
{
    if (high) port->BSRR = (1U << pin);
    else      port->BSRR = (1U << (pin + 16U));
}

static bool gpio_read(GPIO_TypeDef *port, uint32_t pin)
{
    return ((port->IDR >> pin) & 1U) != 0U;
}

/*
 * This starter assumes the core clock is 16 MHz after reset. If your startup
 * code changes the clock, update SystemCoreClock and SysTick_Config accordingly.
 * Delay loop is intentionally simple and should be calibrated for production.
 */
static void timebase_init(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    /* PA0/PA2 outputs for trigger; PA1/PA3 inputs for echo. */
    gpio_mode(GPIOA, TRIG_A_PIN, 1U);
    gpio_mode(GPIOA, TRIG_B_PIN, 1U);
    gpio_mode(GPIOA, ECHO_A_PIN, 0U);
    gpio_mode(GPIOA, ECHO_B_PIN, 0U);

    gpio_output_type_push_pull(GPIOA, TRIG_A_PIN);
    gpio_output_type_push_pull(GPIOA, TRIG_B_PIN);
    gpio_speed(GPIOA, TRIG_A_PIN, 2U);
    gpio_speed(GPIOA, TRIG_B_PIN, 2U);
    gpio_pull(GPIOA, ECHO_A_PIN, 0U);
    gpio_pull(GPIOA, ECHO_B_PIN, 0U);

    gpio_write(GPIOA, TRIG_A_PIN, false);
    gpio_write(GPIOA, TRIG_B_PIN, false);

    /* PB8/PB9 alternate function 4 for I2C1. */
    gpio_mode(GPIOB, 8U, 2U);
    gpio_mode(GPIOB, 9U, 2U);
    GPIOB->OTYPER |= (1U << 8U) | (1U << 9U); /* open-drain */
    gpio_speed(GPIOB, 8U, 2U);
    gpio_speed(GPIOB, 9U, 2U);
    gpio_pull(GPIOB, 8U, 0U);
    gpio_pull(GPIOB, 9U, 0U);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFU << 0U) | (0xFU << 4U)))
                  | (4U << 0U) | (4U << 4U);

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;
}

/* ---------- I2C1 register-level routines, standard-mode 100 kHz ---------- */
static void i2c1_init(void)
{
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0U;
    I2C1->CR2 = 16U;         /* Assumes PCLK1 = 16 MHz; adjust if clock differs. */
    I2C1->CCR = 80U;         /* 100 kHz when PCLK1 = 16 MHz. */
    I2C1->TRISE = 17U;
    I2C1->CR1 = I2C_CR1_PE;
}

static bool i2c1_write_byte(uint8_t address7, uint8_t data)
{
    uint32_t timeout = 100000U;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout) {}
    if (!timeout) return false;

    I2C1->DR = (uint8_t)((address7 << 1U) | 0U);
    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
    }
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    (void)I2C1->SR2;
    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && --timeout) {}
    if (!timeout) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    I2C1->DR = data;
    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && --timeout) {}
    I2C1->CR1 |= I2C_CR1_STOP;
    return timeout != 0U;
}

static void lcd_expander_write(uint8_t data)
{
    (void)i2c1_write_byte(LCD_I2C_ADDR, (uint8_t)(data | LCD_BACKLIGHT));
}

static void lcd_pulse(uint8_t data)
{
    lcd_expander_write((uint8_t)(data | LCD_ENABLE));
    delay_ms(1U);
    lcd_expander_write((uint8_t)(data & ~LCD_ENABLE));
    delay_ms(1U);
}

static void lcd_send_nibble(uint8_t nibble, bool rs)
{
    uint8_t data = (uint8_t)(nibble & 0xF0U);
    if (rs) data |= LCD_RS;
    lcd_expander_write(data);
    lcd_pulse(data);
}

static void lcd_send(uint8_t value, bool rs)
{
    lcd_send_nibble((uint8_t)(value & 0xF0U), rs);
    lcd_send_nibble((uint8_t)((value << 4U) & 0xF0U), rs);
}

static void lcd_command(uint8_t command)
{
    lcd_send(command, false);
    delay_ms((command == 0x01U || command == 0x02U) ? 2U : 1U);
}

static void lcd_data(uint8_t data)
{
    lcd_send(data, true);
}

static void lcd_init(void)
{
    delay_ms(50U);
    lcd_send_nibble(0x30U, false); delay_ms(5U);
    lcd_send_nibble(0x30U, false); delay_ms(1U);
    lcd_send_nibble(0x30U, false); delay_ms(1U);
    lcd_send_nibble(0x20U, false); delay_ms(1U);
    lcd_command(0x28U); /* 4-bit, 2-line */
    lcd_command(0x0CU); /* display on, cursor off */
    lcd_command(0x06U); /* entry mode */
    lcd_command(0x01U); /* clear */
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0U) ? (uint8_t)(0x80U + col)
                                  : (uint8_t)(0xC0U + col);
    lcd_command(address);
}

static void lcd_print(const char *s)
{
    while (*s) lcd_data((uint8_t)*s++);
}

static void lcd_print_u32(uint32_t value)
{
    char buf[11];
    uint32_t i = 0U;
    if (value == 0U) {
        lcd_data('0');
        return;
    }
    while (value && i < sizeof(buf)) {
        buf[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (i > 0U) lcd_data((uint8_t)buf[--i]);
}

/* ---------- HC-SR04 pulse measurement ---------- */
/*
 * Uses SysTick milliseconds for a bounded echo wait. Distance resolution is
 * coarse; for better accuracy use a free-running hardware timer at 1 MHz.
 */
static bool ultrasonic_read_cm(uint32_t trig_pin, uint32_t echo_pin, uint32_t *cm)
{
    uint32_t start_ms;
    uint32_t echo_start_ms;
    uint32_t echo_end_ms;
    uint32_t wait_start;

    gpio_write(GPIOA, trig_pin, false);
    delay_ms(1U);
    gpio_write(GPIOA, trig_pin, true);

    /* Approximately 10 us pulse using a simple loop; tune for the clock. */
    for (volatile uint32_t i = 0U; i < 120U; ++i) __NOP();
    gpio_write(GPIOA, trig_pin, false);

    wait_start = millis();
    while (!gpio_read(GPIOA, echo_pin)) {
        if ((uint32_t)(millis() - wait_start) >= 30U) return false;
    }
    echo_start_ms = millis();
    while (gpio_read(GPIOA, echo_pin)) {
        if ((uint32_t)(millis() - echo_start_ms) >= 30U) return false;
    }
    echo_end_ms = millis();
    start_ms = echo_end_ms - echo_start_ms;
    *cm = (start_ms * 1000U) / 58U; /* Approximation; ms quantization is coarse. */
    return true;
}

static bool sensor_detected(uint32_t trig, uint32_t echo)
{
    uint32_t distance_cm = 0U;
    return ultrasonic_read_cm(trig, echo, &distance_cm) && distance_cm < DETECT_CM;
}

/* ---------- Direction-based occupancy counting ---------- */
static void update_counting(bool a_detected, bool b_detected)
{
    uint32_t now = millis();

    if (g_sequence != SEQ_IDLE &&
        (uint32_t)(now - g_sequence_start_ms) > SEQUENCE_MS) {
        g_sequence = SEQ_IDLE;
    }

    if ((uint32_t)(now - g_last_count_ms) < COOLDOWN_MS) return;

    if (g_sequence == SEQ_IDLE) {
        if (a_detected && !b_detected) {
            g_sequence = SEQ_A_FIRST;
            g_sequence_start_ms = now;
        } else if (b_detected && !a_detected) {
            g_sequence = SEQ_B_FIRST;
            g_sequence_start_ms = now;
        }
        return;
    }

    if (g_sequence == SEQ_A_FIRST && b_detected) {
        g_in_count++;
        g_inside_count = g_in_count - g_out_count;
        g_last_count_ms = now;
        g_sequence = SEQ_IDLE;
    } else if (g_sequence == SEQ_B_FIRST && a_detected) {
        g_out_count++;
        if (g_out_count > g_in_count) {
            g_out_count = g_in_count;
        }
        g_inside_count = g_in_count - g_out_count;
        g_last_count_ms = now;
        g_sequence = SEQ_IDLE;
    }
}

static void display_counts(void)
{
    lcd_set_cursor(0U, 0U);
    lcd_print("IN:");
    lcd_print_u32(g_in_count);
    lcd_print(" OUT:");
    lcd_print_u32(g_out_count);
    lcd_print("   ");

    lcd_set_cursor(1U, 0U);
    lcd_print("INSIDE:");
    lcd_print_u32(g_inside_count);
    lcd_print("       ");
}

int main(void)
{
    timebase_init();
    gpio_init();
    i2c1_init();
    lcd_init();

    lcd_set_cursor(0U, 0U);
    lcd_print("Auditorium Count");
    delay_ms(800U);
    lcd_command(0x01U);

    while (1) {
        /* Sequential polling is simple but can miss closely spaced events. */
        bool a = sensor_detected(TRIG_A_PIN, ECHO_A_PIN);
        bool b = sensor_detected(TRIG_B_PIN, ECHO_B_PIN);
        update_counting(a, b);
        display_counts();
        delay_ms(LOOP_PERIOD_MS);
    }
}

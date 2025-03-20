#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"  // For PWM control
#include "driver/gpio.h"  // For GPIO control

// Motor 1 pins
#define MOTOR1_IN1_PIN    GPIO_NUM_3   // IN1 pin for Motor 1
#define MOTOR1_IN2_PIN    GPIO_NUM_4   // IN2 pin for Motor 1
#define MOTOR1_ENABLE_PIN GPIO_NUM_17  // Enable pin for Motor 1 PWM

// Motor 2 pins
#define MOTOR2_IN3_PIN    GPIO_NUM_11  // IN3 pin for Motor 2 (new)
#define MOTOR2_IN4_PIN    GPIO_NUM_12  // IN4 pin for Motor 2 (new)
#define MOTOR2_ENABLE_PIN GPIO_NUM_18  // Enable pin for Motor 2 PWM (new)

#define MOTOR_PWM_FREQ    5000                // Frequency in Hz for PWM
#define MOTOR_PWM_MODE    LEDC_LOW_SPEED_MODE // Shared mode for both motors
#define MOTOR_PWM_TIMER   LEDC_TIMER_0        // Shared timer
#define MOTOR_PWM_RES     LEDC_TIMER_10_BIT   // PWM resolution (10-bit)
#define MAX_DUTY_CYCLE    1023                // Maximum duty cycle for 10-bit

// PWM channels
#define MOTOR1_PWM_CHANNEL LEDC_CHANNEL_0     // Channel for Motor 1
#define MOTOR2_PWM_CHANNEL LEDC_CHANNEL_1     // Channel for Motor 2

void configure_motors(void) {
    // Configure GPIO for Motor 1 direction control
    esp_rom_gpio_pad_select_gpio(MOTOR1_IN1_PIN);
    gpio_set_direction(MOTOR1_IN1_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(MOTOR1_IN2_PIN);
    gpio_set_direction(MOTOR1_IN2_PIN, GPIO_MODE_OUTPUT);

    // Configure GPIO for Motor 2 direction control
    esp_rom_gpio_pad_select_gpio(MOTOR2_IN3_PIN);
    gpio_set_direction(MOTOR2_IN3_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(MOTOR2_IN4_PIN);
    gpio_set_direction(MOTOR2_IN4_PIN, GPIO_MODE_OUTPUT);

    // Configure PWM timer (shared for both motors)
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = MOTOR_PWM_MODE,
        .duty_resolution  = MOTOR_PWM_RES,
        .timer_num        = MOTOR_PWM_TIMER,
        .freq_hz          = MOTOR_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    // Configure PWM channel for Motor 1
    ledc_channel_config_t motor1_pwm_channel = {
        .gpio_num       = MOTOR1_ENABLE_PIN,
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = MOTOR1_PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&motor1_pwm_channel);

    // Configure PWM channel for Motor 2
    ledc_channel_config_t motor2_pwm_channel = {
        .gpio_num       = MOTOR2_ENABLE_PIN,
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = MOTOR2_PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&motor2_pwm_channel);
}

void set_motor_directions(void) {
    // Motor 1: Clockwise
    gpio_set_level(MOTOR1_IN1_PIN, 0);
    gpio_set_level(MOTOR1_IN2_PIN, 1);
    printf("Motor 1 Direction: Clockwise\n");

    // Motor 2: Clockwise
    gpio_set_level(MOTOR2_IN3_PIN, 1);
    gpio_set_level(MOTOR2_IN4_PIN, 0);
    printf("Motor 2 Direction: Clockwise\n");
}

void set_motor_speeds(int duty_cycle) {
    // Set PWM duty cycle for Motor 1
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR1_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR1_PWM_CHANNEL);
    printf("Motor 1 Duty Cycle: %d\n", duty_cycle);

    // Set PWM duty cycle for Motor 2
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR2_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR2_PWM_CHANNEL);
    printf("Motor 2 Duty Cycle: %d\n", duty_cycle);
}

void stop_motors(void) {
    // Stop Motor 1
    gpio_set_level(MOTOR1_IN1_PIN, 0);
    gpio_set_level(MOTOR1_IN2_PIN, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR1_PWM_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR1_PWM_CHANNEL);
    printf("Motor 1 Stopped\n");

    // Stop Motor 2
    gpio_set_level(MOTOR2_IN3_PIN, 0);
    gpio_set_level(MOTOR2_IN4_PIN, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR2_PWM_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR2_PWM_CHANNEL);
    printf("Motor 2 Stopped\n");
}

void app_main(void) {
    configure_motors();
    set_motor_directions();

    int duty_cycle = 700;
    set_motor_speeds(duty_cycle);

    vTaskDelay(pdMS_TO_TICKS(4000));
    stop_motors();

    vTaskDelay(pdMS_TO_TICKS(1000)); // Idle
}

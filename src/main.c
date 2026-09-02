/*
 * Dual-Mode Autonomous Navigation Robot (ESP32) — Plain C / ESP-IDF port
 * ------------------------------------------------------------------------
 * Ported from the original Arduino (C++) sketch to plain C using the
 * ESP-IDF framework (FreeRTOS + native ESP32 drivers). No Arduino core
 * dependency: uses the gpio, ledc (PWM), and uart drivers directly.
 * Arduino's setup()/loop() split is replaced with a single app_main()
 * that runs its own while(1) loop.
 *
 * Modes (toggled with a push button):
 *   1. OBSTACLE AVOIDANCE - ultrasonic-based wandering
 *   2. VOICE CONTROL      - commands from an offline VR module over UART
 *
 * A simple timing-based dead-reckoning log allows a Return-to-Home (RTH)
 * maneuver that replays the path in reverse.
 *
 * Build: standard ESP-IDF project (idf.py build / idf.py flash monitor)
 *
 * Author: Mahi Raghuvanshi
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "ROBOT";

/* ---------------- Pin Definitions ---------------- */
#define ENA_PIN   25
#define IN1_PIN   26
#define IN2_PIN   27
#define ENB_PIN   33
#define IN3_PIN   14
#define IN4_PIN   12

#define TRIG_PIN  5
#define ECHO_PIN  18

#define MODE_BUTTON_PIN 4

/* Voice module UART */
#define VR_UART_NUM   UART_NUM_2
#define VR_TX_PIN     17
#define VR_RX_PIN     16
#define VR_BAUD_RATE  9600
#define VR_BUF_SIZE   256

/* PWM (LEDC) config for motor speed, replacing Arduino's analogWrite() */
#define PWM_FREQ_HZ      5000
#define PWM_RES          LEDC_TIMER_8_BIT   /* 0-255 duty, matches analogWrite range */
#define PWM_TIMER        LEDC_TIMER_0
#define PWM_MODE         LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL_A    LEDC_CHANNEL_0
#define PWM_CHANNEL_B    LEDC_CHANNEL_1

/* Ultrasonic timeout, matches the original 30ms pulseIn() timeout */
#define ULTRASONIC_TIMEOUT_US 30000

/* Voice command IDs (as trained on the VR module — adjust to your training set) */
typedef enum {
    VR_NONE = -1,
    VR_FORWARD = 0,
    VR_BACKWARD = 1,
    VR_LEFT = 2,
    VR_RIGHT = 3,
    VR_STOP = 4,
    VR_RETURN_HOME = 5
} voice_command_t;

/* Operating modes */
typedef enum {
    MODE_OBSTACLE_AVOIDANCE = 0,
    MODE_VOICE_CONTROL = 1
} robot_mode_t;

static robot_mode_t current_mode = MODE_OBSTACLE_AVOIDANCE;
static bool last_button_state = true; /* pulled-up = true (HIGH) when not pressed */

/* ---------------- Odometry (for Return-to-Home) ---------------- */
typedef struct {
    float heading_deg;
    uint32_t duration_ms;
} path_segment_t;

#define MAX_PATH_SEGMENTS 200
static path_segment_t path_log[MAX_PATH_SEGMENTS];
static int path_length = 0;
static float current_heading_deg = 0.0f; /* 0 = "home-facing" reference */
static bool returning_home = false;

/* ---------------- Forward Declarations ---------------- */
static void drive_forward(void);
static void drive_backward(void);
static void turn_left(void);
static void turn_right(void);
static void stop_motors(void);
static void set_motor_speed(uint32_t duty_a, uint32_t duty_b);
static long read_ultrasonic_cm(void);
static void log_segment(float heading, uint32_t duration_ms);
static void orient_to_heading(float target_heading_deg);
static void execute_return_to_home(void);
static void run_obstacle_avoidance(void);
static void run_voice_control(void);
static voice_command_t read_voice_command(void);
static void handle_mode_button(void);
static void gpio_setup(void);
static void pwm_setup(void);
static void uart_setup(void);

/* ---------------- Peripheral Setup ---------------- */
static void gpio_setup(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) |
                         (1ULL << IN3_PIN) | (1ULL << IN4_PIN) |
                         (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_conf);

    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << MODE_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_conf);

    gpio_set_level(TRIG_PIN, 0);
    stop_motors();
}

/* ENA/ENB pins are driven via LEDC, not plain GPIO — LEDC claims the pin
 * mode itself, which is why they're excluded from gpio_setup() above. */
static void pwm_setup(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_a = {
        .gpio_num = ENA_PIN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_A,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_a);

    ledc_channel_config_t channel_b = {
        .gpio_num = ENB_PIN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_B,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_b);
}

/* Replaces HardwareSerial VoiceSerial(2) from the Arduino version */
static void uart_setup(void)
{
    uart_config_t uart_conf = {
        .baud_rate = VR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(VR_UART_NUM, &uart_conf);
    uart_set_pin(VR_UART_NUM, VR_TX_PIN, VR_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(VR_UART_NUM, VR_BUF_SIZE * 2, 0, 0, NULL, 0);
}

/* ---------------- Mode Switching ---------------- */
static void handle_mode_button(void)
{
    bool reading = (gpio_get_level(MODE_BUTTON_PIN) != 0);

    if (!reading && last_button_state) {
        current_mode = (current_mode == MODE_OBSTACLE_AVOIDANCE)
                            ? MODE_VOICE_CONTROL
                            : MODE_OBSTACLE_AVOIDANCE;
        stop_motors();
        ESP_LOGI(TAG, "Switched mode to: %s",
                 current_mode == MODE_OBSTACLE_AVOIDANCE ? "OBSTACLE AVOIDANCE" : "VOICE CONTROL");
        vTaskDelay(pdMS_TO_TICKS(300)); /* debounce */
    }
    last_button_state = reading;
}

/* ---------------- Obstacle Avoidance Mode ---------------- */
static void run_obstacle_avoidance(void)
{
    long distance_cm = read_ultrasonic_cm();

    if (distance_cm > 25 || distance_cm == -1) {
        drive_forward();
        log_segment(current_heading_deg, 200);
        vTaskDelay(pdMS_TO_TICKS(200));
    } else {
        stop_motors();
        vTaskDelay(pdMS_TO_TICKS(100));

        drive_backward();
        log_segment(current_heading_deg + 180.0f, 300);
        vTaskDelay(pdMS_TO_TICKS(300));

        /* Turn away from obstacle (right turn as default avoidance behavior) */
        turn_right();
        current_heading_deg += 60.0f;
        log_segment(current_heading_deg, 400);
        vTaskDelay(pdMS_TO_TICKS(400));
        stop_motors();
    }
}

/* ---------------- Voice Control Mode ---------------- */
static void run_voice_control(void)
{
    voice_command_t cmd = read_voice_command();
    if (cmd == VR_NONE) return;

    switch (cmd) {
        case VR_FORWARD:
            drive_forward();
            log_segment(current_heading_deg, 500);
            break;
        case VR_BACKWARD:
            drive_backward();
            log_segment(current_heading_deg + 180.0f, 500);
            break;
        case VR_LEFT:
            turn_left();
            current_heading_deg -= 30.0f;
            log_segment(current_heading_deg, 300);
            break;
        case VR_RIGHT:
            turn_right();
            current_heading_deg += 30.0f;
            log_segment(current_heading_deg, 300);
            break;
        case VR_STOP:
            stop_motors();
            break;
        case VR_RETURN_HOME:
            ESP_LOGI(TAG, "Voice command: RETURN HOME received.");
            returning_home = true;
            break;
        default:
            break;
    }
}

/* Reads a recognized command byte from the VR module over UART, if any
 * is currently available (non-blocking — matches Serial.available()
 * behavior from the Arduino version). The Elechouse VR3 module sends a
 * single byte corresponding to the trained command index on a match. */
static voice_command_t read_voice_command(void)
{
    uint8_t data = 0;
    int len = uart_read_bytes(VR_UART_NUM, &data, 1, 0);
    if (len > 0 && data <= 5) {
        return (voice_command_t)data;
    }
    return VR_NONE;
}

/* ---------------- Return-to-Home ---------------- */
static void execute_return_to_home(void)
{
    ESP_LOGI(TAG, "Executing Return-to-Home...");
    for (int i = path_length - 1; i >= 0; i--) {
        float reverse_heading = path_log[i].heading_deg + 180.0f;
        orient_to_heading(reverse_heading);
        drive_forward();
        vTaskDelay(pdMS_TO_TICKS(path_log[i].duration_ms));
        stop_motors();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    path_length = 0;
    current_heading_deg = 0.0f;
    returning_home = false;
    ESP_LOGI(TAG, "Return-to-Home complete.");
}

/* Rotates the robot to approximately face the target heading.
 * Simplified proportional turn based on heading error and timing. */
static void orient_to_heading(float target_heading_deg)
{
    float error = target_heading_deg - current_heading_deg;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (fabsf(error) < 5.0f) return;

    if (error > 0) {
        turn_right();
    } else {
        turn_left();
    }
    vTaskDelay(pdMS_TO_TICKS((int)(fabsf(error) * 4))); /* empirically tuned ms-per-degree */
    stop_motors();
    current_heading_deg = target_heading_deg;
}

static void log_segment(float heading, uint32_t duration_ms)
{
    if (path_length >= MAX_PATH_SEGMENTS) return;
    path_log[path_length].heading_deg = heading;
    path_log[path_length].duration_ms = duration_ms;
    path_length++;
}

/* ---------------- Ultrasonic Sensor ----------------
 * Replaces Arduino's pulseIn() with a manual microsecond-timed poll
 * using esp_timer_get_time(), since pulseIn() is Arduino-core only. */
static long read_ultrasonic_cm(void)
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - wait_start > ULTRASONIC_TIMEOUT_US) return -1;
    }

    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - pulse_start > ULTRASONIC_TIMEOUT_US) return -1;
    }
    int64_t pulse_end = esp_timer_get_time();

    int64_t duration_us = pulse_end - pulse_start;
    return (long)(duration_us / 58); /* convert to cm */
}

/* ---------------- Motor Control ---------------- */
static void set_motor_speed(uint32_t duty_a, uint32_t duty_b)
{
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, duty_a);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, duty_b);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
}

static void drive_forward(void)
{
    gpio_set_level(IN1_PIN, 1); gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 1); gpio_set_level(IN4_PIN, 0);
    set_motor_speed(200, 200);
}

static void drive_backward(void)
{
    gpio_set_level(IN1_PIN, 0); gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 0); gpio_set_level(IN4_PIN, 1);
    set_motor_speed(200, 200);
}

static void turn_left(void)
{
    gpio_set_level(IN1_PIN, 0); gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 1); gpio_set_level(IN4_PIN, 0);
    set_motor_speed(180, 180);
}

static void turn_right(void)
{
    gpio_set_level(IN1_PIN, 1); gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 0); gpio_set_level(IN4_PIN, 1);
    set_motor_speed(180, 180);
}

static void stop_motors(void)
{
    gpio_set_level(IN1_PIN, 0); gpio_set_level(IN2_PIN, 0);
    gpio_set_level(IN3_PIN, 0); gpio_set_level(IN4_PIN, 0);
    set_motor_speed(0, 0);
}

/* ---------------- Entry Point ----------------
 * ESP-IDF has no setup()/loop() split — app_main() runs once at boot,
 * so all one-time setup happens here before an explicit while(1) loop
 * takes over the role of Arduino's loop(). */
void app_main(void)
{
    gpio_setup();
    pwm_setup();
    uart_setup();

    ESP_LOGI(TAG, "Dual-Mode Autonomous Robot Initialized");
    ESP_LOGI(TAG, "Mode: OBSTACLE AVOIDANCE (default)");

    while (1) {
        handle_mode_button();

        if (returning_home) {
            execute_return_to_home();
        } else if (current_mode == MODE_OBSTACLE_AVOIDANCE) {
            run_obstacle_avoidance();
        } else {
            run_voice_control();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); /* yield to FreeRTOS scheduler / other tasks */
    }
}

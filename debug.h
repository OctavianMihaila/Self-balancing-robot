#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define GREEN_LED_PIN PB0
#define BLUE_LED_PIN PB1
#define RED_LED_PIN PB2
#define EXTERN_WHITE_LED PB4
#define EXTERN_YELLOW_LED PB5
#define VOLTAGE_THRESHOLD1 12.3
#define VOLTAGE_THRESHOLD2 12.25
#define VOLTAGE_THRESHOLD3 12.2
#define VOLTAGE_THRESHOLD4 12.0

#define DISPLAY_ADDR 0x27
void init_debug_voltage_LEDs();
void blink_green_LED(float voltage);
void blink_led_once(uint8_t pin);
void blink_green_LED_5_digit(int adcValue);
void collect_gyro_angle_samples(float gyro_angle);
void display_gyro_angle_samples();
#include <avr/io.h>
#include <util/delay.h>

#define GREEN_LED_PIN PB0
#define BLUE_LED_PIN PB1
#define BLUE_LED_PIN2 PB2
#define RED_LED_PIN3 PB3
#define WHITE_LED_PIN4 PB4
#define INTERRUPT_PIN PB5
#define VOLTAGE_THRESHOLD1 12.3
#define VOLTAGE_THRESHOLD2 12.25
#define VOLTAGE_THRESHOLD3 12.2
#define VOLTAGE_THRESHOLD4 12.0

void init_debug_voltage_LEDs();
void debug_voltage_LEDs(float voltage);
void blink_green_LED(float voltage);
void blink_led_once(uint8_t pin);
void blink_green_LED_5_digit(float adcValue);
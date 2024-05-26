#include "debug.h"

#define RESISTOR1 10 // kOhm
#define RESISTOR2 4.7 // kOhm
#define CALIBRATION 3.185 // Calibration factor

// declare a global array to store 10 gyro_angle samples
float gyro_angle_samples[10];
int count_samples = 0;

void init_debug_voltage_LEDs() {
  // Set PB0, PB1, PB2 as outputs
  DDRB |= (1 << GREEN_LED_PIN) | (1 << BLUE_LED_PIN) |  (1 << RED_LED_PIN) | (1 << EXTERN_WHITE_LED) | (1 << EXTERN_YELLOW_LED);
}

void blink_green_LED(float adcValue) {
  float voltage = adcValue * 5.0 / 1023.0 * CALIBRATION; // Calculate voltage from ADC value

  float battery_voltage = voltage * (RESISTOR1 + RESISTOR2) / RESISTOR2; // Calculate battery voltage

  int blink_count_slow = battery_voltage; // Number of blinks for slow interval (3.7V -> 3 blinks)
  int blink_count_fast = (battery_voltage - blink_count_slow) * 10; // Number of blinks for fast interval (3.7V -> 7 blinks)

  // Blink the green LED with slow interval
  for (int i = 0; i < blink_count_slow; i++) {
    PORTB |= (1 << GREEN_LED_PIN); // Turn on Green LED
    _delay_ms(1000);
    PORTB &= ~(1 << GREEN_LED_PIN); // Turn off Green LED
    _delay_ms(1000);
  }

  // blink the blue, once
  PORTB |= (1 << RED_LED_PIN); // Turn on Blue LED
  _delay_ms(1000); // 1 second interval
  PORTB &= ~(1 << RED_LED_PIN); // Turn off Blue LED
  
  // Blink the green LED with fast interval
  for (int i = 0; i < blink_count_fast; i++) {
    PORTB |= (1 << GREEN_LED_PIN); // Turn on Green LED
    _delay_ms(1000); // 1 second interval
    PORTB &= ~(1 << GREEN_LED_PIN); // Turn off Green LED
    _delay_ms(1000); // 1 second interval
  }
}

void blink_led_once(uint8_t pin) { // only for B port !
  PORTB |= (1 << pin); // Turn on Green LED
  _delay_ms(500); // 1 second interval
  PORTB &= ~(1 << pin); // Turn off Green LED
  _delay_ms(500); // 1 second interval
}

void blink_green_LED_5_digit(int value) {
  if (value < 0) {
    // turn white led on
    PORTB |= (1 << EXTERN_WHITE_LED);
    value = -value;
  } else {
    // turn white led off
    PORTB &= ~(1 << EXTERN_WHITE_LED);
  }


  // Separate each digit of the battery voltage (5-digit number)
  int digit1 = (int)(value / 10000) % 10;
  int digit2 = (int)(value / 1000) % 10;
  int digit3 = (int)(value / 100) % 10;
  int digit4 = (int)(value / 10) % 10;
  int digit5 = (int)value % 10;

  // Blink the green LED for each digit with a 1-second interval
  for (int i = 0; i < digit1; i++) {
    blink_led_once(GREEN_LED_PIN);
  }
  blink_led_once(RED_LED_PIN); // Blink the blue LED once between digits

  for (int i = 0; i < digit2; i++) {
    blink_led_once(GREEN_LED_PIN);
  }
  blink_led_once(RED_LED_PIN); // Blink the blue LED once between digits

  for (int i = 0; i < digit3; i++) {
    blink_led_once(GREEN_LED_PIN);
  }
  blink_led_once(RED_LED_PIN); // Blink the blue LED once between digits

  for (int i = 0; i < digit4; i++) {
    blink_led_once(GREEN_LED_PIN);
  }
  blink_led_once(RED_LED_PIN); // Blink the blue LED once between digits

  for (int i = 0; i < digit5; i++) {
    blink_led_once(GREEN_LED_PIN);
  }
}

void collect_gyro_angle_samples(float gyro_angle) {
  // add new sample to the array
  if (count_samples < 10) {
    gyro_angle_samples[count_samples] = gyro_angle;
    count_samples++;
  }
}

void display_gyro_angle_samples() {
  // display the samples
  for (int i = 0; i < 10; i++) {
    blink_green_LED_5_digit(gyro_angle_samples[i]);
    _delay_ms(4000);
  }
}


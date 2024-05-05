#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "stepper_control.h"
#include "debug.h"
#include "mpu6050.h"
#include "battery_manager.h"
#include "main.h"

#define RELAY_PIN PC1
#define POWER_SUPPLY_PIN PD3

volatile uint8_t adcValue = 0;

// Timer0 is responsible for generating interrupts every 25 ms
// that are used in order to mentain the robot in equilibrium.
void timer0_init() {
  // Set Timer/Counter0 to CTC (Clear Timer on Compare Match) mode
  TCCR0A |= (1 << WGM01);
  // Set prescaler to 1024.
  TCCR0B |= (1 << CS02) | (1 << CS00);
  // Set compare match value for 25 ms interrupt at 16MHz CPU clock and 1024 prescaler
  OCR0A = 62;
  // Enable Timer/Counter0 compare match interrupt
  TIMSK0 |= (1 << OCIE0A);
}

// Timer1 is responsible for battery voltage monitoring.
void timer1_init() {
    // Set Timer/Counter1 to CTC (Clear Timer on Compare Match) mode
    TCCR1B |= (1 << WGM12);
    // Set prescaler to 2048
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // Set compare match value to max value. Frequently interrupts are not needed. 
    OCR1A = 65535;
    // Enable Timer/Counter1 compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER0_COMPA_vect) {
  // TODO:
}

// Timer interrupt service routine
ISR(TIMER1_COMPA_vect) {
  adcValue = collect_10_samples(PC3);

  display_battery_voltage(adcValue);
}

// External interrupt service routine for power button press
ISR(INT1_vect) {
  // Prevent multiple interrupts by wainting for the power button to be released
  while (!(PIND & (1 << POWER_SUPPLY_PIN))) {
    _delay_ms(100);
  }

  PORTB ^= (1 << RED_LED_PIN3);
  PORTC ^= (1 << RELAY_PIN);
}

void init_external_interrupt_PD3() {
  // I want the interrupt to trigger when i press the power button. Only once per press
  EICRA |= (1 << ISC11); // Falling edge of INT1 generates an interrupt request
  EIMSK |= (1 << INT1); // Enable INT1
}

void init_power_supply_pins()
{
  DDRD &= ~(1 << POWER_SUPPLY_PIN);
  DDRC |= (1 << RELAY_PIN);
  
  // pull-up resistor for POWER_SUPPLY_PIN
  PORTD |= (1 << POWER_SUPPLY_PIN);
}

int main() {
  sei(); // Enable global interrupts

  init_debug_voltage_LEDs();
  init_battery_manager();
  init_external_interrupt_PD3();
  init_power_supply_pins();
  init_dir_and_step_pins();
  timer1_init();
  twi_init();
  init_mpu6050();

  while (1) {
    // _delay_ms(1000);
    rotate_cw(DIR_PIN_LEFT, STEP_PIN_LEFT, DIR_PIN_RIGHT, STEP_PIN_RIGHT); // Rotate clockwise

    // int16_t x_gyro = read_x_gyro();
    // blink_green_LED_5_digit(x_gyro); // Blink green LED for 5-digit number

    // _delay_ms(10000);
    
    // debug_voltage_LEDs(adcValue); // Debug voltage LEDs

    // _delay_ms(1000);
  }

  return 0;
}
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "stepper_control.h"
#include "debug.h"
#include "mpu6050.h"
#include "battery_manager.h"
#include "movement_controller.h"

#define RELAY_PIN PB3
#define POWER_SUPPLY_PIN PD3

volatile uint8_t adcValue = 0;
volatile int8_t is_moving = -1;
volatile float pid_result = 0;
volatile int battery_intterupt_counter = 0;
volatile uint8_t moving_state = 0;

// DEBUG VARS:
int count_iter = 0;

// Timer0 is responsible for generating interrupts every 25 ms
// that are used in order to mentain the robot in equilibrium.
void timer0_init() {
  // Set Timer/Counter0 to CTC (Clear Timer on Compare Match) mode
  TCCR0A |= (1 << WGM01);
  // Set prescaler to 1024.
  TCCR0B |= (1 << CS02) | (1 << CS00);
  // Set compare match value for 25 us interrupt at 16MHz CPU clock and 1024 prescaler
  OCR0A = 64;
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
  if (pid_result != 0) {
    generate_pulses(pid_result);
  }
}

// Timer interrupt service routine
ISR(TIMER1_COMPA_vect) {



  if (battery_intterupt_counter % 10 == 0) {
    adcValue = collect_10_samples(PC3);
    
    display_battery_voltage(adcValue, moving_state);
  }

  battery_intterupt_counter++;
}

// External interrupt service routine for power button press.
ISR(INT1_vect) {
  _delay_ms(100); // Debounce the button press

  // turn blue led on
  // PORTB ^= (1 << BLUE_LED_PIN);

  if (!(PIND & (1 << POWER_SUPPLY_PIN))) {
    // Turn off the power supply
    PORTB ^= (1 << RELAY_PIN);

    if (is_moving == -1) {
      is_moving = 0;
    } else {
      is_moving = !is_moving;
    }

    // wait for the button to be released
    while (!(PIND & (1 << POWER_SUPPLY_PIN))) {
      _delay_ms(10);
    }
  }
}

void init_external_interrupt_PD3() {
  // I want the interrupt to trigger when i press the power button. Only once per press
  EICRA |= (1 << ISC11); // Falling edge of INT1 generates an interrupt request
  EIMSK |= (1 << INT1); // Enable INT1
}

void init_power_supply_pins()
{
  DDRD &= ~(1 << POWER_SUPPLY_PIN);
  DDRB |= (1 << RELAY_PIN);
  
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
  timer0_init();
  twi_init();
  init_mpu6050();
  gyro_calibration();

  while (1) {

    // stop_motors(STEP_PIN_LEFT, STEP_PIN_RIGHT);
    // _delay_ms(5000);
    // rotate_acw(DIR_PIN_LEFT, STEP_PIN_LEFT, DIR_PIN_RIGHT, STEP_PIN_RIGHT, 2000);

    // rotate_cw(DIR_PIN_LEFT, STEP_PIN_LEFT, DIR_PIN_RIGHT, STEP_PIN_RIGHT, 2000);


    // _delay_ms(5000);

    // --------------  PID IMPLMENTATION ----------------
    float acc_angle = calculate_acc_angle(read_z_accel());
    float gyro_angle;

    if (is_moving == -1 && acc_angle > -2 && acc_angle < 2) {
      gyro_angle = acc_angle;
      is_moving = 1;
      // TODO: Turn some LED ON. Probably the one that is used for calibration as well. BLUE
      // PORTB ^= (1 << BLUE_LED_PIN);
      // turn on yellow led
      PORTB ^= (1 << EXTERN_YELLOW_LED);
    }



    float gyro_y =  read_y_gyro();
    gyro_angle = adjust_gyro_angle(gyro_angle, gyro_y);


    // TODO: DRIFT correction may be needed if the robot loses balance after a while.

       pid_result = calculate_pid_result(acc_angle);
    // pid_result = calculate_pid_result(gyro_angle);
    // blink_green_LED_5_digit(pid_result);

    // Skip one iteration if the robot is straight. Prevents lose of balance when almost straight.
    if (pid_result < 10 && pid_result > -10) {
      pid_result = 0;
    }

    // // Stop the engines if the robot falls.
    // if ((gyro_angle < -45 || gyro_angle > 45) && is_moving) {
    //   // cut the power supply as well
    //   PORTB &= ~(1 << RELAY_PIN);
    //   // toggle blue led
    //   PORTB ^= (1 << BLUE_LED_PIN);
    //   is_moving = 0;

    // }

    // // wait for reset in a while loop 
    // while (!is_moving) {
    //   continue;

    //   // toggle white led
    //   PORTB ^= (1 << EXTERN_WHITE_LED);
    //   _delay_ms(1000);

    // }
  }

  return 0;
}
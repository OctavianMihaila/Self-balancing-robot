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

#define FINAL_ANGLE_OFFSET 71
#define DRIFT_GYRO_CORRECTION 0.9996
#define DRIFT_ACC_CORRECTION 0.0004
#define FILTER_PREVIOUS_ANGLE_WEIGHT 0.9
#define FILTER_NEW_ANGLE_WEIGHT 0.1

volatile uint8_t adcValue = 0;
volatile int battery_intterupt_counter = 0;
volatile int last_interrupt_time = 0;
const unsigned long debounce_delay = 2000; // in milliseconds
volatile unsigned long millis_count = 0;
int started = 0;

volatile long gyro_x_calib;
volatile long gyro_y_calib;
volatile long gyro_z_calib;
volatile float angle_y = 0;
volatile float angle_y_result = 0;
volatile float angle_y_acc = 0;
volatile long magnitude = 0;

volatile float pid_result = 0;
volatile float prev_pid_error = 0;

ISR(TIMER2_COMPA_vect) {
    millis_count++;
}

void timer2_init() {
    // Set the Timer Mode to CTC (Clear Timer on Compare Match)
    TCCR2A |= (1 << WGM21);
    
    OCR2A = 249;  // Assuming a prescaler of 64 for 1ms interval
    
    // Set the prescaler to 64 and start the timer
    TCCR2B |= (1 << CS22);
    
    // Enable Output Compare Match A Interrupt
    TIMSK2 |= (1 << OCIE2A);
    
    // Initialize Counter
    TCNT2 = 0;
    
    // Enable global interrupts
    sei();
}

unsigned long millis() {
    unsigned long millis_return;
    
    // Ensure this cannot be disrupted.
    cli();
    millis_return = millis_count;
    sei();
    
    return millis_return;
}

// Timer0 is responsible for generating interrupts every 20 us
// that are used in order to mentain the robot in equilibrium.
void timer0_init() {
  // Set Timer/Counter0 to CTC (Clear Timer on Compare Match) mode
  TCCR0A |= (1 << WGM01);
  // Set prescaler to 1024.
  TCCR0B |= (1 << CS02) | (1 << CS00);
  // Set compare match value for 20 us interrupt at 16MHz CPU clock and 1024 prescaler
  OCR0A = 39;
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

ISR(TIMER1_COMPA_vect) {
  if (battery_intterupt_counter % 10 == 0) {
    adcValue = collect_10_samples(PC3);
    
    display_battery_voltage(adcValue);
  }

  battery_intterupt_counter++;
}

// External interrupt service routine for power button press.
ISR(INT1_vect) {
  int current_time = millis();
  
  // turn blue led on
  PORTB |= (1 << BLUE_LED_PIN);

  // Check if the required debounce delay has passed
  if ((current_time - last_interrupt_time) > debounce_delay) {
    // Update the last interrupt time
    last_interrupt_time = current_time;
    
    // Check if the power supply button is pressed
    if (!(PIND & (1 << POWER_SUPPLY_PIN))) {
      // Turn the power supply on. This can be change to toggle and the button will
      // be used as a turn on/off button. But something strange happens due to the
      // fact that the button is located around the wire that is connected to the
      // extern interrupt pin and due to electromagnetic interference the interrupt triggers.
      PORTB |= (1 << RELAY_PIN);

      // Wait for the button to be released
      while (!(PIND & (1 << POWER_SUPPLY_PIN))) {
        _delay_ms(10);
      }
    }
  }
}

void init_external_interrupt_PD3() {
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
  timer2_init();
  timer1_init();
  timer0_init();
  twi_init();
  init_mpu6050();
  gyro_calibration(&gyro_x_calib, &gyro_y_calib, &gyro_z_calib);

  // The angle that the MPU6050 has traveled during a loop iteration
  float travel_angle = calculate_travelled_angle();
  // This is used because arduino asin function is working with radians.
  float degrees_to_radians_coeff = calculate_degrees_to_radians_coeff();

  while (1) {
    long acc_x = read_x_accel();
    long acc_y = read_y_accel();
    long acc_z = read_z_accel();
    int16_t gyro_x = read_x_gyro();

    gyro_x -= gyro_x_calib;
    
    // Add the traveled angle since the last loop iteration.
    angle_y += gyro_x * travel_angle;
    
    // Overall acceleration experienced by the robot (euclidian norm).
    magnitude = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

    // Adjust the angle of the robot based on the acceleration.
    angle_y_acc = asin((float)acc_y/magnitude)* degrees_to_radians_coeff;
    
    // Gyro drift correction.
    if(started) {
      angle_y = angle_y * DRIFT_GYRO_CORRECTION + angle_y_acc * DRIFT_ACC_CORRECTION;
    }
    else{ // When starting, set the gyro angle to the accelerometer angle.
      angle_y = angle_y_acc;
      started = 1;
    }
    
    // Using this complementary filter to reduse vibrations.
    // If not used it will go too fast and overshoot.
    angle_y_result = angle_y_result * FILTER_PREVIOUS_ANGLE_WEIGHT + angle_y * FILTER_NEW_ANGLE_WEIGHT;

    // The angle that the robot percepts as being the equilibrium point.
    float reference_angle_value = angle_y_result - FINAL_ANGLE_OFFSET;

    // Used in order to easily find the equilibrium point.
    if (reference_angle_value < 0.5 && reference_angle_value > -0.5) {
      PORTB |= (1 << GREEN_LED_PIN);
    } else {
      PORTB &= ~(1 << GREEN_LED_PIN);
    }

    pid_result = calculate_pid_result(reference_angle_value, prev_pid_error);

    // If the robot is in equilibrium we don't do any movements.
    if (pid_result < 5 && pid_result > -5) {
      pid_result = 0;
    }
  }

  return 0;
}
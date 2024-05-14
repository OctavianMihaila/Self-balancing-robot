#include <avr/io.h>
#include <util/delay.h>

#define STEP_PIN_LEFT (1 << PD4)
#define DIR_PIN_LEFT (1 << PD5)
#define STEP_PIN_RIGHT (1 << PD6)
#define DIR_PIN_RIGHT (1 << PD7)

void init_dir_and_step_pins();
void rotate_cw(uint8_t dir_pin_1, uint8_t step_pin_1, uint8_t dir_pin_2, uint8_t step_pin_2, uint16_t steps);
void rotate_acw(uint8_t dirPin1, uint8_t stepPin1, uint8_t dirPin2, uint8_t stepPin2, uint16_t steps);
void stop_motors(uint8_t step_pin_1, uint8_t step_pin_2);
void generate_pulses(float pid_result);
#include "stepper_control.h"

void init_dir_and_step_pins() {
    DDRD = DIR_PIN_LEFT | STEP_PIN_LEFT | DIR_PIN_RIGHT | STEP_PIN_RIGHT;
    // Set pulse pins to low
    PORTD &= ~STEP_PIN_LEFT;
    PORTD &= ~STEP_PIN_RIGHT;
}

void rotate_cw(uint8_t dir_pin_1, uint8_t step_pin_1, uint8_t dir_pin_2, uint8_t step_pin_2, uint16_t steps) {
    unsigned int i;
    PORTD = 0x00; //reset PORTD
    //send High pulse for clockwise direction
    PORTD |= dir_pin_1; // stepper 1
    PORTD |= ~dir_pin_2; // stepper 2
    //send 1600 pulses to rotate One full cycle
    for(i = 0 ; i < steps; i++){
        PORTD |= step_pin_1;
        PORTD |= step_pin_2;
        _delay_ms(0.5);
        PORTD &= ~step_pin_1;
        PORTD &= ~step_pin_2;
        _delay_ms(0.5);
    }
}

void rotate_acw(uint8_t dirPin1, uint8_t stepPin1, uint8_t dirPin2, uint8_t stepPin2, uint16_t steps) {
    unsigned int i;
    PORTD = 0x00; //reset PORTD

    //send low pulse for anti-clockwise direction
    PORTD |= ~dirPin1; // stepper 1
    // should rotate in opposite direction
    PORTD |= dirPin2; // stepper 2
    //send 1600 pulses to rotate One full cycle
    for(i = 0 ; i < steps; i++){
        PORTD |= stepPin1;
        PORTD |= stepPin2;
        _delay_ms(0.5);
        PORTD &= ~stepPin1;
        PORTD &= ~stepPin2;
        _delay_ms(0.5);
    }
}

void stop_motors(uint8_t step_pin_1, uint8_t step_pin_2) {
    PORTD &= ~step_pin_1;
    PORTD &= ~step_pin_2;
    _delay_ms(10); // Short delay to ensure motors stop
}

// Generates pulses based on the PID result. This will be called in the interrupt service routine that is called every 25us.
void generate_pulses(float pid_result) {
    // TODO: If not working well the number of steps cand be increased based on the pid_result.
    // Also print the values of the pid_result to see the bounderies and the it can be mapped into a number of steps.

    // For now, generate 1 pulse for each interrupt.
    if (pid_result > 0) {
        rotate_cw(DIR_PIN_LEFT, STEP_PIN_LEFT, DIR_PIN_RIGHT, STEP_PIN_RIGHT, 1);
    } else if (pid_result < 0) {
        rotate_acw(DIR_PIN_LEFT, STEP_PIN_LEFT, DIR_PIN_RIGHT, STEP_PIN_RIGHT, 1);
    }
}
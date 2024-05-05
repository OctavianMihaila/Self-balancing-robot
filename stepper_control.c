#include "stepper_control.h"

void init_dir_and_step_pins() {
    DDRD = DIR_PIN_LEFT | STEP_PIN_LEFT | DIR_PIN_RIGHT | STEP_PIN_RIGHT;
    // Set pulse pins to low
    PORTD &= ~STEP_PIN_LEFT;
    PORTD &= ~STEP_PIN_RIGHT;
}

void rotate_cw(uint8_t dir_pin_1, uint8_t step_pin_1, uint8_t dir_pin_2, uint8_t step_pin_2) {
    unsigned int i;
    //send High pulse for clockwise direction
    PORTD |= ~dir_pin_1; // stepper 1
    PORTD |= ~dir_pin_2; // stepper 2
    //send 1600 pulses to rotate One full cycle
    for(i = 0 ; i < 1600; i++){
        PORTD |= step_pin_1;
        PORTD |= step_pin_2;
        _delay_ms(0.5);
        PORTD &= ~step_pin_1;
        PORTD &= ~step_pin_2;
        _delay_ms(0.5);
    }
}

void rotate_acw(uint8_t dirPin1, uint8_t stepPin1, uint8_t dirPin2, uint8_t stepPin2) {
    unsigned int i;
    //send low pulse for anti-clockwise direction
    PORTD &= ~dirPin1; // stepper 1
    PORTD &= ~dirPin2; // stepper 2
    //send 1600 pulses to rotate One full cycle
    for(i = 0 ; i < 1600; i++){
        PORTD |= stepPin1;
        PORTD |= stepPin2;
        // _delay_ms(2);
        PORTD &= ~stepPin1;
        PORTD &= ~stepPin2;
        // _delay_ms(2);
    }
}
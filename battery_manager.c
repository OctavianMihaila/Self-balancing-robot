#include "battery_manager.h"
#include "debug.h"

void init_battery_manager() {
  // Set PB0, PB1, PB2, PB3, PB4 as outputs
  DDRB |=  (1 << HALF_BATTERY_LED) | (1 << DISCHARGED_BATTERY_LED);

  // Set up the ADC for pin PC3
    // ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX1); // Select AVcc as reference, input channel 3 (PC3)
    ADMUX = (1 << REFS0) | (1 << MUX1) | (1 << MUX0); 
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, set prescaler to 128
}

int readADC(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // select the channel
  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete

  // switch the state (on/off) for the INTERRUPT_PIN when interrupt occurs
  // PORTB ^= (1 << INTERRUPT_PIN);

 
  return ADC;
}

uint8_t collect_10_samples(uint8_t channel) {
  uint16_t adcSum = 0;
  for (uint8_t i = 0; i < 10; i++) {
    adcSum += readADC(channel);
  }

  return adcSum / 10;
}

void display_battery_voltage(float adcValue) {
    float readed_voltage = adcValue * (5.0 / 1023.0) * CALIBRATION;

    float battery_voltage = readed_voltage * (RESISTOR1 + RESISTOR2) / RESISTOR2;

    // blink_green_LED_5_digit(battery_voltage); // Blink green LED for 5-digit number

    // Control blue LED based on voltage
    if (battery_voltage < VOLTAGE_THRESHOLD_1) {
        PORTB |= (1 << HALF_BATTERY_LED); // Turn on Red LED if voltage is below threshold
    } else {
        PORTB &= ~(1 << HALF_BATTERY_LED); // Turn off Red LED otherwise
    }

    // battery voltage less than VOLTAGETHRESHOLD2 -> TURN ON second blue LED
    if (battery_voltage < VOLTAGE_THRESHOLD_2) {
        PORTB |= (1 << DISCHARGED_BATTERY_LED); // Turn on Red LED if voltage is below threshold
    } else {
        PORTB &= ~(1 << DISCHARGED_BATTERY_LED); // Turn off Red LED otherwise
    }
}
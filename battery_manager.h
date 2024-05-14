#include <avr/io.h>
#include <util/delay.h>

#define DISCHARGED_BATTERY_LED PB2
#define VOLTAGE_THRESHOLD_1 11.5
#define RESISTOR1 10 // kOhm
#define RESISTOR2 4.7 // kOhm
#define CALIBRATION 3.1 // Calibration factor

void init_battery_manager();
int readADC(uint8_t channel);
uint8_t collect_10_samples(uint8_t channel);
void display_battery_voltage(float adcValue, uint8_t moving_state);

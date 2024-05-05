#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi.h"

#define gyro_address 0x68

void init_mpu6050();
void get_gyro_data(uint8_t *data, uint8_t reg);
int16_t read_x_gyro();
int16_t combineBytes(uint8_t highByte, uint8_t lowByte);
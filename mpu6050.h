#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi.h"

#define GYRO_ADDRESS 0x68
#define PI 3.142
#define HALF_CIRCLE 180
#define acc_z_calibration 2000 // TODO: ADJUST IF NEEDED. READ FROM 3F AND 40 REGISTERS.
#define LSB_SENSITIVITY_4G 8192 // This coresponds to 1g acceleration when the sensitivity is set to 4g.
#define FS_SEL_0_TYP 131 // Gyro ouputs 131 when set to 250 degrees per second. (In this case it rotates at 1 degree per second).
#define LOOP_DURATION 2 // TODO: Measure the final loop and ajust. For now let's say it is 2ms.


void init_mpu6050();
void get_mpu_6050_data(uint8_t *data, uint8_t reg);
int16_t read_x_gyro();
int16_t read_y_gyro();
int16_t read_z_accel();
int16_t combineBytes(uint8_t highByte, uint8_t lowByte);
void gyro_calibration();
float calculate_acc_angle(int16_t z_accel);
float adjust_gyro_angle(uint16_t old_gyro_angle, int16_t gyro_y);
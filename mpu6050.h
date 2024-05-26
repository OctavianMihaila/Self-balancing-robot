#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi.h"

#define GYRO_ADDRESS 0x68
#define PI 3.142
#define HALF_CIRCLE 180
#define acc_z_calibration 2000 // This is the value that the accelerometer outputs when it is at rest.
#define LSB_SENSITIVITY_4G 8192 // This coresponds to 1g acceleration when the sensitivity is set to 4g.4
// Gyro ouputs 131 when set to 250 degrees per second. (In this case it rotates at 1 degree per second).
#define FS_SEL_0_TYP 131 

#define ACCEL_Z_OFFSET 393.0
// Due to the fact that the fact that the angular velocity is 6 degrees / second,
// one rotation will take 60 seconds.
#define ONE_ROT_TIME_COMPLETION 60.0
#define REFRESH_RATE 250.0 // Hz. This means we can read data from MPU6050 every 1 / 250 = 4ms.

void init_mpu6050();
void get_mpu_6050_data(uint8_t *data, uint8_t reg);
int16_t read_x_gyro();
int16_t read_y_gyro();
int16_t read_z_gyro();
int16_t read_x_accel();
int16_t read_y_accel();
int16_t read_z_accel();
int16_t combineBytes(uint8_t highByte, uint8_t lowByte);
void gyro_calibration(volatile long *gyro_x_calib, volatile long *gyro_y_calib, volatile long *gyro_z_calib);
int16_t calibrate_gyro_x(int16_t gyro_x);
int16_t calibrate_gyro_y(int16_t gyro_y);
float calculate_travelled_angle();
float calculate_degrees_to_radians_coeff();
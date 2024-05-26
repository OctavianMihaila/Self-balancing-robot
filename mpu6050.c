#include "mpu6050.h"
#include "debug.h"

void init_mpu6050() {
    twi_start();
    twi_write(GYRO_ADDRESS << 1);
    twi_write(0x6B); // PWR_MGMT_1 register.
    twi_write(0x00); // Activate the gyro.
    twi_stop();

    twi_start();
    twi_write(GYRO_ADDRESS << 1);
    twi_write(0x1B); // GYRO_CONFIG register.
    twi_write(0x08); // Set the full scale range to 500 degrees per second.
    twi_stop();

    twi_start();
    twi_write(GYRO_ADDRESS << 1);
    twi_write(0x1C); // ACCEL_CONFIG register.
    twi_write(0x10); // Set the full scale range to 8g. (Accesleration will not be over +/- 8g)
    twi_stop();
}

void get_mpu_6050_data(uint8_t *data, uint8_t reg) {
    twi_start();

    twi_write(GYRO_ADDRESS << 1);
    twi_write(reg);
    twi_start();
    twi_write((GYRO_ADDRESS << 1) | 1);
    twi_read_nack(data);

    twi_stop();
}

int16_t read_x_gyro()
{
    uint8_t x_gyro_high, x_gyro_low;
    
    get_mpu_6050_data(&x_gyro_high, 0x43);
    get_mpu_6050_data(&x_gyro_low, 0x44);
    int16_t x_gyro = combineBytes(x_gyro_high, x_gyro_low);

    return x_gyro;
}

int16_t read_y_gyro()
{
    uint8_t y_gyro_high, y_gyro_low;
    
    get_mpu_6050_data(&y_gyro_high, 0x45);
    get_mpu_6050_data(&y_gyro_low, 0x46);
    int16_t y_gyro = combineBytes(y_gyro_high, y_gyro_low);

    return y_gyro;
}

int16_t read_z_gyro()
{
    uint8_t z_gyro_high, z_gyro_low;
    
    get_mpu_6050_data(&z_gyro_high, 0x47);
    get_mpu_6050_data(&z_gyro_low, 0x48);
    int16_t z_gyro = combineBytes(z_gyro_high, z_gyro_low);

    return z_gyro;
}

int16_t read_x_accel()
{
    uint8_t x_accel_high, x_accel_low;
    
    get_mpu_6050_data(&x_accel_high, 0x3B);
    get_mpu_6050_data(&x_accel_low, 0x3C);
    int16_t x_accel = combineBytes(x_accel_high, x_accel_low);

    return x_accel;
}

int16_t read_y_accel()
{
    uint8_t y_accel_high, y_accel_low;
    
    get_mpu_6050_data(&y_accel_high, 0x3D);
    get_mpu_6050_data(&y_accel_low, 0x3E);
    int16_t y_accel = combineBytes(y_accel_high, y_accel_low);

    return y_accel;
}

int16_t read_z_accel()
{
    uint8_t z_accel_high, z_accel_low;
    
    get_mpu_6050_data(&z_accel_high, 0x3F);
    get_mpu_6050_data(&z_accel_low, 0x40);
    int16_t z_accel = combineBytes(z_accel_high, z_accel_low);

    // calibrate the z-acceleration
    z_accel -= acc_z_calibration;

    // Make sure the value is not exceeding LSB_SENSITIVITY_4G value.
    // In this way, we prevent division by zero in calculate_angle function.
    if (z_accel > LSB_SENSITIVITY_4G) {
        z_accel = LSB_SENSITIVITY_4G;
    } else if (z_accel < -LSB_SENSITIVITY_4G) {
        z_accel = -LSB_SENSITIVITY_4G;
    }

    return z_accel;
}

void gyro_calibration(volatile long *gyro_x_calib, volatile long *gyro_y_calib, volatile long *gyro_z_calib) {
    // Collect 2000 samples of x and y gyro data
    for (int i = 0; i < 2000; i++) {
        *gyro_x_calib += read_x_gyro();
        *gyro_y_calib += read_y_gyro();
        *gyro_z_calib += read_z_gyro();
        _delay_us(3); // Used to simulate one iteration of the while loop.
    }

    *gyro_x_calib /= 2000;
    *gyro_y_calib /= 2000;
    *gyro_z_calib /= 2000;

    // Blink blue led 3 times to mark the end of calibration
    for (int i = 0; i < 3; i++) {
        PORTB |= (1 << BLUE_LED_PIN);
        _delay_ms(200);
        PORTB &= ~(1 << BLUE_LED_PIN);
        _delay_ms(200);
    }

}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    // Combine the high byte (left-shifted by 8 bits) with the low byte
    return (int16_t)((highByte << 8) | lowByte);
}

float calculate_travelled_angle() {
    float division_factor = (ONE_ROT_TIME_COMPLETION * ACCEL_Z_OFFSET) / REFRESH_RATE;

    return (1 / REFRESH_RATE) / division_factor;
}

float calculate_degrees_to_radians_coeff() {
    return 1 / (PI / HALF_CIRCLE);
}

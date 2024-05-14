#include "mpu6050.h"
#include "debug.h"

int16_t gyro_x_calib;
int16_t gyro_y_calib;

void init_mpu6050() {
    twi_start();
    twi_write(GYRO_ADDRESS << 1);
    twi_write(0x6B); // PWR_MGMT_1 register.
    twi_write(0x00); // Activate the gyro.
    twi_stop();

    // Low pass filter prevents peaks in data output.
    twi_start();
    twi_write(0x1A); // CONFIG register.
    twi_write(0x03); // Set the low pass filter to 44Hz in order to reduce noise.
    twi_stop();

    // WARNING !!!
    // It seems that the default value of the gyro is 250 degrees per second.
    // Since this is generating an infinite loop, i will not use it for now.

    // twi_start();
    // twi_write(0x1B); // GYRO_CONFIG register.
    // twi_write(0x00); // Set the full scale range to 250 degrees per second.
    // twi_stop();

    twi_start();
    twi_write(0x1C); // ACCEL_CONFIG register.
    twi_write(0x08); // Set the full scale range to 4g. (Accesleration will not be over +/- 4g)
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

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    // Combine the high byte (left-shifted by 8 bits) with the low byte
    return (int16_t)((highByte << 8) | lowByte);
}

void gyro_calibration() {
    int16_t x_gyro = 0;
    int16_t y_gyro = 0;

    // Collect 200 samples of x and y gyro data
    for (int i = 0; i < 200; i++) {
        x_gyro += read_x_gyro();
        y_gyro += read_y_gyro();
        _delay_ms(2); // Used to simulate one iteration of the while loop.
    }

    gyro_x_calib = x_gyro / 200;
    gyro_y_calib = y_gyro / 200;

    // Blink blue led 3 times to mark the end of calibration
    for (int i = 0; i < 3; i++) {
        PORTB |= (1 << BLUE_LED_PIN);
        _delay_ms(200);
        PORTB &= ~(1 << BLUE_LED_PIN);
        _delay_ms(200);
    }

}

// TOOD: Should be called in the main loop.
float calculate_acc_angle(int16_t z_accel) {
    // Calculate the angle of the robot based on the z-acceleration.
    // The angle is calculated in degrees.
    float degree_conversion_factor = HALF_CIRCLE / PI;

    return asin((float)z_accel / LSB_SENSITIVITY_4G) * degree_conversion_factor;
}

// TODO: Should be called in the main loop.
float adjust_gyro_angle(uint16_t old_gyro_angle, int16_t gyro_y) {
    gyro_y -= gyro_y_calib;
    //  blink_green_LED_5_digit(gyro_y);
    float loop_duration = LOOP_DURATION;
    float fs_sel_0_typ = FS_SEL_0_TYP;

    float gyro_movement_factor = 1 / (1 / loop_duration * fs_sel_0_typ);

    return old_gyro_angle + gyro_y * gyro_movement_factor;
}


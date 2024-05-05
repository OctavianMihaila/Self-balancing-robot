#include "mpu6050.h"
#include "debug.h"

void init_mpu6050() {
    twi_start();
    twi_write(gyro_address << 1);
    twi_write(0x6B); // PWR_MGMT_1 register.
    twi_write(0x00); // Activate the gyro.
    twi_stop();

    // This will improve 
    twi_start();
    twi_write(0x1A); // CONFIG register.
    twi_write(0x03); // Set the low pass filter to 44Hz in order to reduce noise.
    twi_stop();

    twi_start();
    twi_write(0x1B); // GYRO_CONFIG register.
    twi_write(0x00); // Set the full scale range to 250 degrees per second.
    twi_stop();

    twi_start();
    twi_write(0x1C); // ACCEL_CONFIG register.
    twi_write(0x08); // Set the full scale range to 4g. (Accesleration will not be over +/- 4g)
    twi_stop();
}

void get_gyro_data(uint8_t *data, uint8_t reg) {
    twi_start();

    twi_write(gyro_address << 1);
    twi_write(reg);
    twi_start();
    twi_write((gyro_address << 1) | 1);
    twi_read_nack(data);

    twi_stop();
}

int16_t read_x_gyro()
{
    uint8_t x_gyro_high, x_gyro_low;
    
    get_gyro_data(&x_gyro_high, 0x43);
    get_gyro_data(&x_gyro_low, 0x44);
    int16_t x_gyro = combineBytes(x_gyro_high, x_gyro_low);

    return x_gyro;
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    // Combine the high byte (left-shifted by 8 bits) with the low byte
    return (int16_t)((highByte << 8) | lowByte);
}

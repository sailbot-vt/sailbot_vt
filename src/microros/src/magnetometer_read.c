#include "common_microros_libs.h"
#include "hardware/i2c.h"
#include <math.h>

// I2C Setup for Pi Pico
#define I2C_PORT i2c0
#define SDA_PIN 4  // Pin 4 for SDA
#define SCL_PIN 5  // Pin 5 for SCL

// Address of the CMPS14 compass on i2c
#define _i2cAddress 0x60

// CMPS14 register definitions
#define CONTROL_Register 0
#define BEARING_Register 2
#define PITCH_Register 4
#define ROLL_Register 5

#define MAGNETX_Register 6
#define MAGNETY_Register 8
#define MAGNETZ_Register 10

#define ACCELEROX_Register 12
#define ACCELEROY_Register 14
#define ACCELEROZ_Register 16

#define GYROX_Register 18
#define GYROY_Register 20
#define GYROZ_Register 22

#define ONE_BYTE   1
#define TWO_BYTES  2
#define FOUR_BYTES 4
#define SIX_BYTES  6

// Variables
int bearing;
signed char pitch;
signed char roll;

float accelScale = 9.80592991914f / 1000.f; // 1 m/s^2
float gyroScale = 1.0f / 16.f;              // 1 Dps

// I2C write function
int i2c_write_byte(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    int result = i2c_write_blocking(I2C_PORT, _i2cAddress, data, 2, false);
    return result;
}

// I2C read function for multiple bytes
int i2c_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
    // Send the register address
    i2c_write_blocking(I2C_PORT, _i2cAddress, &reg, 1, true);
    // Read the response
    int result = i2c_read_blocking(I2C_PORT, _i2cAddress, buffer, length, false);
    return result;
}

// Initialize I2C
void i2c_init_custom() {
    i2c_init(I2C_PORT, 100 * 1000);  // Initialize I2C at 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Get Bearing (2 bytes)
int16_t getBearing() {
    uint8_t buffer[2];
    if (i2c_read_bytes(BEARING_Register, buffer, 2) != 2) {
        return 0;
    }
    int16_t result = ((buffer[0] << 8) | buffer[1]) / 10;
    return result;
}

// Get Pitch (1 byte)
int8_t getPitch() {
    uint8_t buffer;
    if (i2c_read_bytes(PITCH_Register, &buffer, 1) != 1) {
        return 0;
    }
    return (int8_t)buffer;
}

// Get Roll (1 byte)
int8_t getRoll() {
    uint8_t buffer;
    if (i2c_read_bytes(ROLL_Register, &buffer, 1) != 1) {
        return 0;
    }
    return (int8_t)buffer;
}

// Read Accelerometer (6 bytes)
void ReadAccelerator(float *accelX, float *accelY, float *accelZ) {
    uint8_t buffer[6];
    if (i2c_read_bytes(ACCELEROX_Register, buffer, 6) != 6) {
        *accelX = 0;
        *accelY = 0;
        *accelZ = 0;
        return;
    }
    *accelX = ((int16_t)(buffer[0] << 8 | buffer[1])) * accelScale;
    *accelY = ((int16_t)(buffer[2] << 8 | buffer[3])) * accelScale;
    *accelZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * accelScale;
}

// Read Gyroscope (6 bytes)
void ReadGyro(float *gyroX, float *gyroY, float *gyroZ) {
    uint8_t buffer[6];
    if (i2c_read_bytes(GYROX_Register, buffer, 6) != 6) {
        *gyroX = 0;
        *gyroY = 0;
        *gyroZ = 0;
        return;
    }
    *gyroX = ((int16_t)(buffer[0] << 8 | buffer[1])) * gyroScale;
    *gyroY = ((int16_t)(buffer[2] << 8 | buffer[3])) * gyroScale;
    *gyroZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * gyroScale;
}

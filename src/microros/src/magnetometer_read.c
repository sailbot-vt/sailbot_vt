#include "common_microros_libs.h"
#include "hardware/i2c.h"
#include <math.h>

// I2C Setup for Pi Pico
#define I2C_PORT i2c0

// CMPS14 register definitions
#define CONTROL_Register 0x00
#define BEARING_Register 0x02
#define PITCH_Register 0x04
#define ROLL_Register 0x05

#define MAGNETX_Register 0x06
#define MAGNETY_Register 0x08
#define MAGNETZ_Register 0x0A

#define ACCELEROX_Register 0x20
#define ACCELEROY_Register 0x22
#define ACCELEROZ_Register 0x24

#define GYROX_Register 0x25
#define GYROY_Register 0x27
#define GYROZ_Register 0x29

#define ROLLP_Register 0x1C
#define PITCHP_Register 0x1A

#define Calibration_Register 0x1E


#define ONE_BYTE   1
#define TWO_BYTES  2
#define FOUR_BYTES 4
#define SIX_BYTES  6

// Variables
int bearing;
signed char pitch;
signed char roll;

uint8_t _i2cAddress;

float accelScale = 9.80592991914 / 1000.0; // 1 m/s^2
float gyroScale = 1.0 / 16.0;              // 1 Dps
float magnetScale = 1.0;                   // No clue



void i2c_init_custom(uint8_t address) {
    _i2cAddress = address;
    i2c_init(I2C_PORT, 100 * 1000);  // Initialize I2C at 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); // This is what we wanted on the PCB
    gpio_pull_up(SCL_PIN);
}

int i2c_write_byte(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    int result = i2c_write_blocking(I2C_PORT, _i2cAddress, data, TWO_BYTES, false);
    return result;
}

int i2c_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(I2C_PORT, _i2cAddress, &reg, ONE_BYTE, true);
    int result = i2c_read_blocking(I2C_PORT, _i2cAddress, buffer, length, false);
    return result;
}

int16_t getBearing() {
    uint8_t buffer[TWO_BYTES];
    if (i2c_read_bytes(BEARING_Register, buffer, TWO_BYTES) != TWO_BYTES) {
        return -1;
    }
    int16_t result = ((buffer[0] << 8) | buffer[1]);
    return result;
}

int8_t getPitch() {
    uint8_t buffer;
    if (i2c_read_bytes(PITCH_Register, &buffer, ONE_BYTE) != ONE_BYTE) {
        return 0;
    }
    return (int8_t)buffer;
}

int8_t getRoll() {
    uint8_t buffer;
    if (i2c_read_bytes(ROLL_Register, &buffer, ONE_BYTE) != ONE_BYTE) {
        return 0;
    }
    return (int8_t)buffer;
}

void readAccelerator(float *accelX, float *accelY, float *accelZ) {
    uint8_t buffer[SIX_BYTES];
    if (i2c_read_bytes(ACCELEROX_Register, buffer, SIX_BYTES) != SIX_BYTES) {
        *accelX = 0;
        *accelY = 0;
        *accelZ = 0;
        return;
    }
    *accelX = ((int16_t)(buffer[0] << 8 | buffer[1])) * accelScale;
    *accelY = ((int16_t)(buffer[2] << 8 | buffer[3])) * accelScale;
    *accelZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * accelScale;
}

void readGyro(float *gyroX, float *gyroY, float *gyroZ) {
    uint8_t buffer[SIX_BYTES];
    if (i2c_read_bytes(GYROX_Register, buffer, SIX_BYTES) != SIX_BYTES) {
        *gyroX = 0;
        *gyroY = 0;
        *gyroZ = 0;
        return;
    }
    *gyroX = ((int16_t)(buffer[0] << 8 | buffer[1])) * gyroScale;
    *gyroY = ((int16_t)(buffer[2] << 8 | buffer[3])) * gyroScale;
    *gyroZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * gyroScale;
}

void readMagnet(float *magnetX, float *magnetY, float *magnetZ) {
    uint8_t buffer[SIX_BYTES];
    if (i2c_read_bytes(MAGNETX_Register, buffer, SIX_BYTES) != SIX_BYTES) {
        *magnetX = 0;
        *magnetY = 0;
        *magnetZ = 0;
        return;
    }
    *magnetX = ((int16_t)(buffer[0] << 8 | buffer[1])) * magnetScale;
    *magnetY = ((int16_t)(buffer[2] << 8 | buffer[3])) * magnetScale;
    *magnetZ = ((int16_t)(buffer[4] << 8 | buffer[5])) * magnetScale;
}
#include "MPU6050_Sensor.h"

uint8_t MPU6050_Sensor::board_address_read = 0x68;
uint8_t MPU6050_Sensor::power_mgmt_1 = 0x6B;
uint8_t MPU6050_Sensor::gyro_config = 0x1B;
uint8_t MPU6050_Sensor::gyro_xout_h = 0x43;
uint8_t MPU6050_Sensor::gyro_yout_h = 0x45;
uint8_t MPU6050_Sensor::accel_xout_h = 0x3B;
uint8_t MPU6050_Sensor::accel_yout_h = 0x3D;
uint8_t MPU6050_Sensor::accel_zout_h = 0x3F;

void MPU6050_Sensor::begin() {
    Wire.begin();
    Wire.beginTransmission(board_address_read);
    Wire.write(power_mgmt_1); 
    Wire.write(0x08); // Turn off temperature sensor
    Wire.endTransmission(true);
    delay(50);
    Wire.beginTransmission(board_address_read);
    Wire.write(gyro_config);
    Wire.write(0x00); // Configure Gyroscope FS_SEL bits
    Wire.endTransmission(true);
}

float MPU6050_Sensor::getGyroXAxis() {
    Wire.beginTransmission(board_address_read);
    Wire.write(gyro_xout_h);
    Wire.endTransmission(true);
    Wire.requestFrom(board_address_read, 2, true);
    int16_t gyroXRaw = Wire.read() << 8 | Wire.read();
    return gyroXRaw / 131.0f; // Remove LSB unit
}

float MPU6050_Sensor::getGyroYAxis() {
    Wire.beginTransmission(board_address_read);
    Wire.write(gyro_yout_h);
    Wire.endTransmission(true);
    Wire.requestFrom(board_address_read, 2, true);
    int16_t gyroYRaw = Wire.read() << 8 | Wire.read();
    return gyroYRaw / 131.0f; // Remove LSB unit
}

float MPU6050_Sensor::getAccelXAxis() {
    Wire.beginTransmission(board_address_read);
    Wire.write(accel_xout_h);
    Wire.endTransmission(true);
    Wire.requestFrom(board_address_read, 2, true);
    int16_t accelXRaw = Wire.read() << 8 | Wire.read();
    return accelXRaw / 16384.0f; // Remove LSB unit
}


float MPU6050_Sensor::getAccelYAxis() {
    Wire.beginTransmission(board_address_read);
    Wire.write(accel_yout_h);
    Wire.endTransmission(true);
    Wire.requestFrom(board_address_read, 2, true);
    int16_t accelYRaw = Wire.read() << 8 | Wire.read();
    return accelYRaw / 16384.0f; // Remove LSB unit
}

float MPU6050_Sensor::getAccelZAxis() {
    Wire.beginTransmission(board_address_read);
    Wire.write(accel_zout_h);
    Wire.endTransmission(true);
    Wire.requestFrom(board_address_read, 2, true);
    int16_t accelZRaw = Wire.read() << 8 | Wire.read();
    return accelZRaw / 16384.0f; // Remove LSB unit
}
#include "MPU6050_Sensor.h"

uint8_t MPU6050_Sensor::board_address_read = 0x68;
uint8_t MPU6050_Sensor::power_mgmt_1 = 0x6B;
uint8_t MPU6050_Sensor::gyro_xout_h = 0x43;

MPU6050_Sensor::MPU6050_Sensor() {
    Wire.begin();
    Wire.beginTransmission(board_address_read);
    Wire.write(power_mgmt_1); 
    Wire.write(0x08);// Turn off temperature sensor
    Wire.endTransmission();
}

float MPU6050_Sensor::getGyroXAxis() {
    Wire.beginTransmission(board_address_read);
    Wire.write(gyro_xout_h);
    Wire.endTransmission(true);
    Wire.requestFrom(board_address_read, 2, true);
    int16_t gyroXRaw = Wire.read() << 8 | Wire.read();
    return gyroXRaw / 32.8f; // Remove LSB unit
}

#include <Wire.h>
#include <Arduino.h>

#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H 

class MPU6050_Sensor {

private:

    static uint8_t board_address_read;
    static uint8_t power_mgmt_1;    
    static uint8_t MPU6050_Sensor::gyro_xout_h;

public:

    MPU6050_Sensor();

    float getGyroXAxis();

};

#endif
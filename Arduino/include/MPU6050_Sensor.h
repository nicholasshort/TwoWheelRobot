#include <Wire.h>
#include <Arduino.h>

#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H 

class MPU6050_Sensor {

private:

    static uint8_t board_address_read;
    static uint8_t power_mgmt_1;  
    static uint8_t gyro_config;    
    static uint8_t gyro_xout_h;
    static uint8_t gyro_yout_h;
    static uint8_t accel_xout_h;
    static uint8_t accel_yout_h;
    static uint8_t accel_zout_h;


public:

    void begin();
    float getGyroXAxis();
    float getGyroYAxis();
    float getAccelXAxis();
    float getAccelYAxis();
    float getAccelZAxis();

};

#endif
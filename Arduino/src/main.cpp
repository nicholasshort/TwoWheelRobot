#include <Arduino.h>
#include <Wire.h>
#include "KalmanFilter.h"
#include "MPU6050_Sensor.h"

#define PWM_A 6
#define PWM_B 5
#define IN_1 9
#define IN_2 7
#define IN_3 13
#define IN_4 12


MPU6050_Sensor mpu;

// Variances
float Qo = 0.001;
float Qb = 0.003;
float R = 20;

KalmanFilter kalmanFilter(Qo, Qb, R);

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// State
float* state = (float*)calloc(2, sizeof(float));

// Error variables
float error = 0;
float prevError = 0;
float totalError = 0;

// Angle
float desiredAngle = 0;

// PID Values
float Kp = 500;
float Ki = 15;
float Kd = 700;

// Controller Output
float output = 0;

// Start state variable
bool start = false;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Starting...");

  state[0] = -85;

  mpu.begin();

  // Initialize pinout
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

}

void loop()
{
  timer = millis();

  // Read accelerometer values
  float accelX = mpu.getAccelXAxis();
  float accelY = mpu.getAccelYAxis();
  float accelZ = mpu.getAccelZAxis();

  // Read gyro value
  float gyroY = mpu.getGyroYAxis();

  // Calculate Pitch
  float pitch = -atan(accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

  // Run measurement through kalman filter
  state = kalmanFilter.update(state, pitch, gyroY, timeStep);

  // Error calculations
  error = state[0] - desiredAngle;

  if(abs(error) > 45) start = false;

  if(abs(error) < 1) start = true;

  if(start){ 

    // Integral error
    totalError = totalError + error;

    // Constrain total error
    if(totalError > 500) totalError = 500;
    if(totalError < -500) totalError = -500;
 
    // PID Calculations
    output = Kp*error + Ki*totalError + Kd*(error - prevError);
    output = map(abs(output), 0, 1250, 0, 255);
    output = output > 255 ? 255 : output;

  }

  output = start ? output : 0;
  Serial.println(state[0]);
  prevError = error;
  
  if(error < 0) {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);
    analogWrite(PWM_A, output);
    analogWrite(PWM_B, output);
  }else{
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);
    analogWrite(PWM_A, output);
    analogWrite(PWM_B, output);
  }

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));

}



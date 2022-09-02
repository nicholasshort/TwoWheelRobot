#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

#define PWM_A 6
#define PWM_B 5
#define IN_1 9
#define IN_2 7
#define IN_3 13
#define IN_4 12


MPU6050 mpu;

KalmanFilter kalmanFilter(0.001f, 0.003f, 0.03f);

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
float desiredAngle = -90;

// PID Values
float Kp = 600;
float Ki = 5;
float Kd = 100;

// Controller Output
float output = 0;

// Start state variable
bool start = false;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Starting...");

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  Serial.println("Calibrating Gyro...");
  mpu.calibrateGyro();
  Serial.println("Calibration Complete!");

  // Set threshold sensivty. Default 3.
  mpu.setThreshold(3);

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

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Angle
  state = kalmanFilter.update(state, (state[0] + norm.YAxis * timeStep), norm.YAxis, timeStep);
  
  // Error calculations
  error = state[0] - desiredAngle;

  if(abs(error) > 90) start = false;

  if(abs(error) < 0.5) start = true;

  if(start){ 

    totalError = totalError + error;

    // PID Calculations
    output = Kp*error + Ki*totalError + Kd*(error - prevError);
    output = map(abs(output), 0, 1250, 0, 255);
    output = output > 255 ? 255 : output;

  }

  output = start ? output : 0;
  
  Serial.print("Output: ");
  Serial.println(output);

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

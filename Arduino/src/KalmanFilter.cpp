#include <Arduino.h>
#include "KalmanFilter.h"


KalmanFilter::KalmanFilter(float Qo, float Qb, float R){
    this->Q[0][0] = Qo;
    this->Q[1][1] = Qb;
    this->R = R;

    this->P[0][0] = 0;
    this->P[0][1] = 0;
    this->P[1][0] = 0;
    this->P[1][1] = 0;
}

float* KalmanFilter::predict(float state[], float theta_dot, float dt){
    
    // Update state using previous state (Xk|k-1)
    state[0] = state[0] + dt*(theta_dot - state[1]);
    
    // Update error covariance using previous error covariance (Pk|k-1)
    P[0][0] = P[0][0] + dt*(dt*P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
    P[0][1] = P[0][1] - dt * P[1][1];
    P[1][0] = P[1][0] - dt * P[1][1];
    P[1][1] = P[1][1] + Q[1][1] * dt;
    
    return state;
}

float* KalmanFilter::update(float state[], float theta, float theta_dot, float dt){

    // Get priori state 
    state = predict(state, theta_dot, dt);
    
    // Inovation
    float y = theta - state[0];

    // Inovation Covariance
    float S = P[0][0] + R;

    // Kalman Gain
    float K[2];

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update to current state
    state[0] = state[0] + y * K[0];
    state[1] = state[1] + y * K[1];

    // Update Error Covariance Matrix
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] = P[0][0] - K[0] * P00_temp;
    P[0][1] = P[0][1] - K[0] * P01_temp;
    P[1][0] = P[1][0] - K[1] * P00_temp;
    P[1][1] = P[1][1] - K[1] * P01_temp;

    // Return current state (Xk|k)
    return state;
}
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter{

private:
    // Process and Measurement Noise
    float Q[2][2];
    float R;
    
    // Error Covariance Matrix
    float P[2][2];

    // Preprocess states
    float* predict(float state[], float theta_dot, float dt);
  

public:

    // Variance Parameters
    KalmanFilter(float Qo, float Qb, float R);

    // Update state
    float* update(float state[], float theta, float theta_dot, float dt);

};

#endif
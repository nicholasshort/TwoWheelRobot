# TwoWheelRobot

This is a project I started to learn more about control theory. 

The project consists of three main folders:
1) Arduino Code
2) Hand Derivations for the Linearized State Space model of a Pendulum on a Cart
3) Matlab/Python Pendulum on a Cart Simulation Files

I will try to go over each folder in the remainder of this readme.

# The Robot

The Bill of Materials for this project can be found here: https://docs.google.com/document/d/1V8id3cu9YUFI_mSXT52ZumMh5tL_UX_wbOAgka5fCik/edit?usp=sharing

To control the robot, I use a PID controller, with the help of a Kalman filter (defined in KalmanFilter.h) to filter out the noise from the accelerometer and to factor in gyroscope drift. A good artilce on the fundamentals of the Kalman filter can be found here: https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
One thing to note for my design was that the height of the IMU board from the base of the robot resulted in greater angular accelerations and thus greater noise from the accelerometer. Therefore, the Measurement Noise variance had to be set to a greater value then the generic value reccomended in the article above. 

Here is a video of the robot maintaining balance on youtube: https://www.youtube.com/shorts/AJx97sAAVso

# Hand Derivations

The two wheel robot can be modeled as a pendulum on a cart. I used Newton's Laws to derive the state space model of the pendulum-cart system, and then reduced the number of states to only focus on the angle of the pendulum.

# Matlab and Python Simulations

I used matlab and python to simulate/verify my the state space model from my hand derivations, and to develop a full-state feedback control loop to control the cart and pendulum about a fixed angle and position respectively. Running PendulumCartSim.py will show a small animation of the implementation of this control law.

Here is the animation without any control:
![final_631eb3e16659cb00835e07b7_296242](https://user-images.githubusercontent.com/56266904/189573809-808b3170-8c6b-4a50-8f01-f93935982ef1.gif)

Here is the animation of the linearized system with full state feedback:
![final_631eb2a96fb6b5005e3506c4_549431](https://user-images.githubusercontent.com/56266904/189573871-6fa7416a-b05d-4d48-8bf4-04387191e27d.gif)


Below is an image of the control loop for full state feedback.

![Full State Feedback](https://user-images.githubusercontent.com/56266904/189548020-bbbb1152-c92d-497f-bb80-6247c9aa2ebc.PNG)

Additionally, I also tried to control the system using a PID controller. In "PID_Tuning.m", I implemented a very slow root-locus plot in Matlab to generate proper PID gain values to stabilize the system. An example of the root locus plot can be seen below.

![Root Locus](https://user-images.githubusercontent.com/56266904/189548647-d0ea9e37-3c35-4ff8-b398-48a559acb3de.PNG)

I then used the PID values found from the locus plot to plot the system's step response (Note these are not optimal PID values)

![Step Response](https://user-images.githubusercontent.com/56266904/189548765-66a3a7c9-077d-4b21-a014-2f3554d6f8e2.PNG)



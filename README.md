# TwoWheelRobot

This is a project I started to learn more about control theory. 

The project consists of three main folders:
1) Arduino Code
2) Matlab/Python Pendulum on a Cart Simulation Files
3) Hand Derivations for the Linearized State Space model of a Pendulum on a Cart

# Matlab and Python Simulations

I used matlab and python to simulate/verify my the state space model from my hand derivations, and to develop a full-state feedback control loop to control the cart and  pendulum about a fixed angle and position respectively. Running PendulumCartSim.py will show a small animation of the implementation of this control law. Below is an image of the control loop for full state feedback.

![Full State Feedback](https://user-images.githubusercontent.com/56266904/189548020-bbbb1152-c92d-497f-bb80-6247c9aa2ebc.PNG)

Additionally, I also tried to control the system using a PID controller. In "PID_Tuning.m", I implemented a very slow root-locus plot in Matlab to generate proper PID gain values to stabilize the system. An example of the root locus plot can be seen below.

![Root Locus](https://user-images.githubusercontent.com/56266904/189548647-d0ea9e37-3c35-4ff8-b398-48a559acb3de.PNG)

I then used the PID values found from the locus plot to plot the system's step response (Note these are not optimal PID values)

![Step Response](https://user-images.githubusercontent.com/56266904/189548765-66a3a7c9-077d-4b21-a014-2f3554d6f8e2.PNG)

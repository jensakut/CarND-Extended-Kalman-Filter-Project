# Extended Kalman Filter fusing Radar and Lidar measurement for tracking and object

Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

## How to run the code 
1. mkdir build
2. cd build
3. cmake ..
4. make
Without uWebSocketIO or the simulator, the following lines serve as a test: 
5a. ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output1.txt > input1.log
6a. ./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt output2.txt > input2.log

The detailed instructions may be found [here](https://github.com/jensakut/CarND-Extended-Kalman-Filter-Project/blob/master/TASK.md). 

> Your algorithm will be run against Dataset 1 in the simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]. 

The algorithm beat the target with the following errors: 
x= 0.0967
y= 0.0851
VX=0.3854
VY=0.4243

> Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The sensor fusion algorithm was designed using the codes taught in the preceding lessons. In main.cpp the interface to the simulation is called and the classes are called. 
The kalman-filter.cpp contains the kalman matrices and vectors as well as the update and prediction function for a linear and extended kalman-filter. 
Besides using tools class to compute a specific jacobian for the radar, the kalman-filter is entirely generic and reusable. 
The FusionEKF class uses the Kalman_filter class (without any inheritance) to implement the specific matrices needed for computation. 
The ProcessMeasurement function is the core of the class, preparing the sensor-specific parts H_ and R_ and the measurement-vector and executing the update and measurement steps. 


> Your Kalman Filter algorithm handles the first measurements appropriately.
The first measurement is used for initialization. With the second step, the first full kalman-filter is executed. It doesn't matter, which sensor is used at initialization. 

> Your Kalman Filter can handle radar and lidar measurements.
Both the linear sensor for the positions given by laser and the extended kalman filter equations are used to estimate the systems starte (position and velocity). 


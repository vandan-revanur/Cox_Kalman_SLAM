# Cox_Kalman_SLAM

This repository contains files for Simulation of Simultaneous Localization and mapping (SLAM) for robot equipped with a RPLidar A2 sensor, and electrical motors with inbuilt encoders.

Localalization is realised using a Kalman filter which fuses the dead reckoning position estimate and
laser position measurements to obtain an estimate with reduced error.
The Cox scan matching algorithm is used for the mapping. 
It matches of the laser measurement data with a prior known map,  which aids in updating the robots current estimate.


## Data Description

The data for the simulation are logs from the Lidar sensor and Odometry readings of the dead reckoning. 
These are found in the directory Logs. Lidar_log.csv and Odo_Log.csv.

## Usage 

Run the main.py for the simulation. 
Modify the cox.py if you would like simulate a different map.

![](Media/SLAM.gif)

# Cox_Kalman_SLAM

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3678904.svg)](https://doi.org/10.5281/zenodo.3678904)

## Acknowledgements

I would like to sincerely thank my teammate : Fredrik Svanström, who was majorly involved in the development of the code and supported me in understanding the theory behind concepts such as the Dead reckoning, Cox Algorithm and Kalman Filtering for sensor fusion, and the physical construction of the robot. 

## Description of the repository

This repository contains files for Simulation of Simultaneous Localization and mapping (SLAM) for a robot equipped with a RPLidar A2 sensor, and electrical motors with inbuilt encoders.

The robot looks like this. 

<img src="Media/robot.png" width="324" height="324">

Localization is realised using a Kalman filter which fuses the dead reckoning position estimate and
laser position measurements to obtain an estimate with reduced error.
The Cox scan matching algorithm is used for the mapping, based on the paper [1], matches the laser measurement data with a prior known map, which aids in updating the robots current estimate.



## Data Description

The data for the simulation are logs from the Lidar sensor and Odometry readings of the dead reckoning. 
These are found in the directory Logs. 
* Lidar_log.csv 
* Odo_Log.csv.

## Usage 
 
Run the main.py for the simulation. 
Modify the cox.py if you would like simulate a different map.

In the below gif: 
* The green dot corresponds to the position as estimated by the odometry.
* The yellow dot corresponds to the position as estimated by the odmetry corrected by the Cox scan.
* The black dot corresponds to the Kalman corrected reading.

![](Media/SLAM.gif)

## Citation

If you plan to use this for your research, please consider citing using the following BibTeX:

@software{vandan_revanur_2020_3678904,
  author       = {Vandan Revanur},
  title        = {vandan-revanur/Cox\_Kalman\_SLAM: First Release},
  month        = feb,
  year         = 2020,
  publisher    = {Zenodo},
  version      = {1.0.0},
  doi          = {10.5281/zenodo.3678904},
  url          = {https://doi.org/10.5281/zenodo.3678904}
}

## References

[1] https://ieeexplore.ieee.org/document/75902 :  [PDF](https://github.com/vandan-revanur/Cox_Kalman_SLAM/blob/master/docs/cox1991.pdf)


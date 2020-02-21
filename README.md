# Cox_Kalman_SLAM
A repository containing files of a Simultaneous localization and mapping (SLAM) project. 

Localalization is realised using a Kalman filter which fuses the dead reckoning position estimate and
laser position measurements to obtain an estimate with reduced error.
The Cox scan matching algorithm is used for the mapping. It matches of the laser measurement data with a prior known map, 
which aids in updating the robots current estimate.



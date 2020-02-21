# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 21:39:33 2020

@author: Vandan
"""

import numpy as np

def Cox_LineFit(posX, posY, posA, angs, meas, ):

    # LINEMATRIX for the lines, midpoints and lengths: To describe the map
    # Change this variable according to your map
	# Format: 
	#  1  2  3  4  		5  			6      		7
    # [x1 y1 x2 y2 x_mid_point y_mid_point line_length*0.5]
    LINEMATRIX = np.array([[0, 0, 2425, 0, 1213, 0, 1213],
                           [2425, 0, 2425, 3640, 2425, 1820, 1820],
                           [2425, 3640, 0, 3640, 1213, 3640, 1213],
                           [0, 3640, 0, 0, 0, 1820, 1820]])

    dx = 0.0
    dy = 0.0
    da = 0.0
    PosCov_Cox = np.zeros((3, 3))
    ViCopy = 0

    U = np.array([[0, 1], [-1, 0], [0, -1], [1, 0]])
    ri = np.array([[0], [-2425], [-3640], [0]])

    # Defalut B and PosCov_Cox to be used in test if B is getting bigger
    BOld = [1000000, 1000000, 1000000]
    PosCov_Cox_Old = np.array([[100000, 0, 0],
                  [0, 100000, 0],
                  [0, 0, 100000]])

    n = 1  # Loop counter

    while n < 10:
        posX = posX + dx
        posY = posY + dy
        posA = np.mod(posA + da, 2 * np.pi)

        number_of_laser_measurements = angs.shape[0]
        Xw = np.zeros((number_of_laser_measurements, 5))

        for idx in range(0, number_of_laser_measurements):
            
            # Step 1: Translate and rotate data points
            
            # Lidar Sensor values to Cartesian Coordinates (The x,y are the sensor coordinates)
            alpha = angs[idx]
            r = meas[idx]
            x = r * np.cos(alpha)
            y = r * np.sin(alpha)

            # lidar position on robot
            Sx = 0
            Sy = 0
            Sa = -np.pi / 2
            
            # Lidar Sensor Coordinates to robot coordinates

            R = np.array([[np.cos(Sa), -np.sin(Sa), Sx], [np.sin(Sa), np.cos(Sa), Sy], [0, 0, 1]])
            Xs = np.dot(R, (np.array([x, y, 1])))
            
            
            # Robot Coordinates to World Coordinates
            # Here R is the homogenous transformation matrix and C describes vehicle's position with respect to the world coordinates
            
            CR = np.array([[np.cos(posA), -np.sin(posA), posX], [np.sin(posA), np.cos(posA), posY], [0, 0, 1]])
            
            # unpack[0] refers to the x coordinate of the lidar sensor in world, 
            # and unpack[1] refers to the y coordinate of the lidar sensor in the world
            unpack = np.dot(CR, np.array([Xs[0], Xs[1], 1]))
            
            # Appart from the (x,y,1) we also collect the measurement id: idx, 
            # and we also include a distance rejection threshold as the last element of thenp.array
            
            outlier_rejection_threshold = 10000
            Xw[idx, 0:5] = np.array([idx, unpack[0], unpack[1], unpack[2], outlier_rejection_threshold])
            
            
            # Step 2: Find the target for the data points
            
            
            for i in range(0,4):
                yi = ri[i, 0] - np.dot(U[i, 0:2], Xw[idx, 1:3])
                if np.abs(yi) <= np.abs(Xw[idx, 4]):
                    Xw[idx, 3] = i
                    Xw[idx, 4] = yi

        # To get the median distance value
        Xm = np.abs(Xw[:, 4])
        M = np.median(Xm)

        # Counter for the new Vi matrix (a matrix with outliers removed)
        matrix_counter = 0

        # Reject outliers is done by:
        # 1. Checking the distance from measurement to the line
        # 2. Checking that the distance from the laser point to the 
        # midpoint of the line is not more than 0.5*length of the line
        # This is done since the calculations conciders the lines as
        # infintely long

        # To create the Vi matrix with
        #        0          1       2       3       4       5
        # [nearest_line x-coord y-coord distance normalX normalY]
        Vi = np.empty((0, 6), float)

        # Use measurements that are less than x mm from a wall

        Distance_Rejection_Threshold = 1000 # in mm
        for idx in range(0, Xw.shape[0]):
            if (np.abs(Xw[idx, 4]) <= Distance_Rejection_Threshold) and ((np.sqrt((Xw[idx, 1] - LINEMATRIX[int(Xw[idx, 3]), 4]) ** 2 + (
                    (Xw[idx, 2] - LINEMATRIX[int(Xw[idx, 3]), 5]) ** 2))) <= (LINEMATRIX[int(Xw[idx, 3]), 6])):
                ViTemp = np.array(
                    [[Xw[idx, 3], Xw[idx, 1], Xw[idx, 2], Xw[idx, 4], U[int(Xw[idx, 3]), 0], U[int(Xw[idx, 3]), 1]]])
                Vi = np.append(Vi, ViTemp, axis=0)
                matrix_counter = matrix_counter + 1

        # If the number of measurements are very small we change the distance limit
        # This value was found by trial and error experimentation 
        Min_Measurements_Satisfying_Distance_Criterion = 25 # in mm
        
        # If the number of points satisfying the distance criterion is lesser than the min threshold,
        # then we compare the distances with the median value (M) instead of Distance_Rejection_Threshold
        if matrix_counter < Min_Measurements_Satisfying_Distance_Criterion:
            for idx in range(0, Xw.shape[0]):
                if (np.abs(Xw[idx, 4]) <= M) and ((np.sqrt((Xw[idx, 1] - LINEMATRIX[int(Xw[idx, 3]), 4]) ** 2 + (
                        (Xw[idx, 2] - LINEMATRIX[int(Xw[idx, 3]), 5]) ** 2))) <= (LINEMATRIX[int(Xw[idx, 3]), 6])):
                    ViTemp = np.array([[Xw[idx, 3], Xw[idx, 1], Xw[idx, 2], Xw[idx, 4], U[int(Xw[idx, 3]), 0],
                                     U[int(Xw[idx, 3]), 1]]])
                    Vi = np.append(Vi, ViTemp, axis=0)
                    matrix_counter = matrix_counter + 1

        # A copy of Vi to be used for plotting
        ViCopy = Vi
        
        # Step 3: Setting Up linear equation system
    
        Xi1 = np.zeros((Vi.shape[0], 1))
        Xi2 = np.zeros((Vi.shape[0], 1))
        Xi3 = np.zeros((Vi.shape[0], 1))

        Xi1[:, 0] = Vi[:, 4]
        Xi2[:, 0] = Vi[:, 5]
        R = np.array([[0, -1], [1, 0]])
        vm=  np.array([np.mean(Xw[:, 1]), np.mean(Xw[:, 2])])

        for idx in range(0, Vi.shape[0]):
            ui = np.array([Vi[idx, 4], Vi[idx, 5]])
            vi = np.array([Vi[idx, 1], Vi[idx, 2]]) 
            Xi3[idx, 0] = np.dot(ui, np.dot(R, vi-vm))

        # To find B = [dx dy da]'
        Y = Vi[:, 3]

        # A = array([Xi1, Xi2, Xi3])
        A = np.column_stack((Xi1, Xi2, Xi3))
#        A = np.column_stack((A, Xi3))

        # To calculate B = inv(A'*A)*A'*Y
        B = np.linalg.lstsq(A, Y, rcond=-1)[0]

        # Calculate the covariance matrix (PosCov_Cox)
        # Here pinv is used to account for inversion of near singular matrices
        
        S2 = np.dot((Y - np.dot(A, B)).transpose(), Y - np.dot(A, B)) / (A.shape[0] - 4)
        PosCov_Cox = np.dot(S2, (np.linalg.pinv(np.dot(A.transpose(), A))))
        # print('PosCov_Cox: ', PosCov_Cox)

        # To test if B is getting bigger.
        # If B is getting bigger then stop and use value from last loop
        if np.sqrt(B[0] ** 2 + B[1] ** 2) > np.sqrt(BOld[0] ** 2 + BOld[1] ** 2):
            #print('B getting bigger. Stopping')
            B = [0, 0, 0]
            PosCov_Cox = PosCov_Cox_Old
            n = 100

        # To save B and PosCov_Cox to next loop
        BOld = B
        PosCov_Cox_Old = PosCov_Cox
        
        #Step 4: Add latest contribution to the change in position and angle
        dx = dx + B[0]
        dy = dy + B[1]
        da = da + B[2]

        # Check if the loop should be stopped: that is if the process has converged
        if (np.sqrt(B[0] ** 2 + B[1] ** 2) < 15) and (np.abs(B[2] < 1 * np.pi / 180)):
            #print('Fit stop')
            n = 100

        # Emergency stop: If the change in x,y or A is more than 1000mm then reject those values
        # That is if the Cox algorithm says the robot has moved by 1m in 10ms then it is not feasible since our robot
        # moves at quite low speeds
        if (np.abs(dx) > 1000) or (np.abs(dy) > 1000) or (np.abs(da) > 2):
            dx = 0
            dy = 0
            da = 0
            PosCov_Cox = np.array([[10000, 0, 0], [0, 10000, 0], [0, 0, 100]])
            #print('E stop')
            n = 100

        n = n + 1  # Update loop counter

    return [dx, dy, da, PosCov_Cox, ViCopy]
    
    
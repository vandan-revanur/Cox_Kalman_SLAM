# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 12:19:36 2020

@author: Vandan
"""

import numpy as np
from numpy import array
from numpy.linalg import pinv
import pandas as pd
import matplotlib.pyplot as plt

# Import the Cox Line fitting function
from Cox import Cox_LineFit

if __name__ == "__main__":
    
    # Read the Odometry and Lidar Logs
    Lidar_df = pd.read_csv('Logs/Lidar_Log.csv' )
    Odo_df = pd.read_csv('Logs/Odo_Log.csv')
 
    # x,y,A values initialized from the Odometry log
    
    posX = Odo_df.iloc[0,1]
    posY = Odo_df.iloc[0,2]
    posA = Odo_df.iloc[0,3]
    
    # Lists for printing the Odometry path taken 
    Odo_Curr_PosX_list = []
    Odo_Curr_PosY_list =[]
    Odo_Curr_PosA_list = []
    
    # Initialize the cox corrected values with the initial odometry x,y,A values
    Cox_Corrected_posX = [posX]
    Cox_Corrected_posY = [posY]
    Cox_Corrected_posA = [posA]
    
    # Grouping the measurements into scans
    # Here 150 measurements is grouped as one scan
    # So 150 angle and distance measurements is one scan
    
    i =0
    angsdeg = []
    distances = []
    
    while (i+150<=Lidar_df.shape[0] ):
        angsdeg.append(Lidar_df.iloc[i:i+150,1].to_list())
        distances.append(Lidar_df.iloc[i:i+150,2].to_list())
        i+=150
    
    # Plotting the Odometry (x,y) in Green and Cox Corrected (x,y) in yellow and the fused Kalman (x,y) in black
    fig = plt.figure()
    image_count = 0
    
    for odo_row, angle_degree, distance in zip(Odo_df.values,angsdeg,distances ):
        
        # Collected the x,y,A and PosCov_Odo from the Odometry log
        Odo_Curr_PosX_list.append( odo_row[1])
        Odo_Curr_PosY_list.append(odo_row[2])
        Odo_Curr_PosA_list.append( odo_row[3])
        PosCov_Odo= np.array([ [odo_row[4], odo_row[5], odo_row[6] ],
                              [odo_row[7], odo_row[8], odo_row[9]] ,
                              [odo_row[10], odo_row[11], odo_row[12]] ])
        
        # Converting the angles to radians and arena setup related necessary transformation of angles
        angle_radian =[]        
        for i in angle_degree:
            
            # Here the -ve sign is reverse the direction of the lidar angle measureemnts
            # because the lidar sensor is running clockwise(compass angles), and robot is using mathematical
            # angles , and then +90 is to compensate for the mounting angle of the lidar
            angle_radian.append(np.mod(-np.deg2rad(i) + np.pi / 2, 2 * np.pi))
            
        # Converting lists to numpy arrays    
        angle_radian=array(angle_radian)
        yi= array(distance)  
            
        # Plotting the walls 
        LDX = []
        LDY = []
        for i, j in zip(yi, angle_radian):
            LDX.append(int(posX + i * np.cos(j+posA-np.pi/2)))
            LDY.append(int(posY + i * np.sin(j+posA-np.pi/2)))
            
        # Calling the Cox Line Fitting function
        [dx, dy, da, PosCov_Cox, ViCopy] = Cox_LineFit(posX, posY, posA, angle_radian, yi)
        
        # Updating the position with the corrections obtained from the Cox Line fit
        posX = posX + dx
        posY = posY + dy
        posA = posA + da
    
        # Appending to cox array for plotting
        Cox_Corrected_posX.append(posX)
        Cox_Corrected_posY.append(posY)
        Cox_Corrected_posA.append(posA)
        
        dPos_hat_Cox = np.array([dx, dy, da])
    
        # Start of Kalman part: Fusing the estimates from the Odometry and the Cox Line fitting
        dPos_Kalman =np.matmul(np.matmul(PosCov_Odo,(pinv(PosCov_Cox+PosCov_Odo))),dPos_hat_Cox)
        PosCov_Kalman = pinv(pinv(PosCov_Cox)+pinv(PosCov_Odo))
        
        DX_Kalman =  ((dPos_Kalman).tolist())[0] 
        DY_Kalman =  ((dPos_Kalman).tolist())[1]
        DA_Kalman =  ((dPos_Kalman).tolist())[2]
        
        # Updating the Odometry Position with the corrections from Kalman filter
        Kalman_Corrected_PosX = Odo_Curr_PosX_list[-1] + DX_Kalman
        Kalman_Corrected_PosY = Odo_Curr_PosY_list[-1] + DY_Kalman
        Kalman_Corrected_PosA = Odo_Curr_PosA_list[-1] + DA_Kalman
        Kalman_Corrected_PosCov = PosCov_Kalman
        
        plt.clf()
        plt.plot(Odo_Curr_PosX_list,Odo_Curr_PosY_list, 'y.', ms =35 )
        plt.plot(LDX, LDY, 'b*') # To plot all available measruements in the Lidar scan in blue
        plt.plot(ViCopy[:, 1], ViCopy[:, 2], 'r*') # To plot only those used points from the scans which satisfy the 1000mm criterion
        plt.plot(Cox_Corrected_posX, Cox_Corrected_posY, 'c.', ms = 17) # To plot current position and history of robot positions
        plt.plot(Kalman_Corrected_PosX,Kalman_Corrected_PosY, 'k.',  ms =10)
        plt.axis([-2000, 5000, -1200, 4800])
        plt.draw()
        plt.pause(0.1)
        
#        plt.savefig('{}.png'.format(image_count), bbox_inches='tight')
#        image_count+=1
        



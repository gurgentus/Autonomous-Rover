#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = np.eye(2)
        self.R_t = np.eye(3)
    
        self.P = 1000 * np.eye(3);
        self.state = np.array([0.3, 0.375, 0])
    
    def normAngle(self, angle):
        newAngle = angle
        while newAngle <= -np.pi:
            newAngle = newAngle + 2*np.pi
        while newAngle > np.pi:
            newAngle = newAngle - 2*np.pi
        return newAngle
   

    def prediction(self, v, omega2, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        dt = 0
      
        if self.last_time != None:
            dt = imu_meas[4][0]-self.last_time
        self.last_time = imu_meas[4][0]
        print("dt", dt)
      
        th = self.state[2]
       
        dfdx = np.array([ [1, 0, -dt*v*np.sin(th)], [0,1, dt*v*np.cos(th)], [0,0,1]])
       
        dfdn = np.array([[dt*np.cos(th), 0], [dt*np.sin(th), 0], [0,dt]])
 	omega = -imu_meas[3] 
        
	upd = np.array([v*np.cos(th), v*np.sin(th), omega])
        print("imu", omega,"upd", upd)
        self.state = self.state + dt*upd
        self.P = dfdx.dot(self.P).dot(dfdx.transpose()) + dfdn.dot(self.Q_t).dot(dfdn.transpose())
        # if self.state[2] > 2*np.pi:
        #     self.state[2] = self.state[2]-2*np.pi
        #self.state[2] = self.normAngle(self.state[2])
        print("state0", self.state)
	

    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        tagLoc = self.markers[self.markers[:,3] == z_t[0][3]]
     
        theta_w = tagLoc[0][2]
        x_w = tagLoc[0][0]
        y_w = tagLoc[0][1]
        H1 = np.array([[np.cos(theta_w), -np.sin(theta_w), x_w], [np.sin(theta_w), np.cos(theta_w), y_w], [0, 0, 1]])
        
        theta_r = z_t[0][2]
        x_r = z_t[0][0]
        y_r = z_t[0][1]
        H2 = np.array([[np.cos(theta_r), -np.sin(theta_r), x_r], [np.sin(theta_r), np.cos(theta_r), y_r], [0, 0, 1]])
        H = H1.dot(np.linalg.inv(H2))
        x = H[0][2]
        y = H[1][2]
        th = np.arctan2(H[1][0],H[0][0])
        arr = np.array([x,y,th])
        print("guess",arr)
        print("state", self.state)
        self.K = self.P.dot(np.linalg.inv(self.R_t + self.P))      
      
        if (np.absolute(arr[2]-self.state[2]) > np.pi):
            self.state[2] = self.state[2] - 2*np.pi
        if (np.absolute(arr[2]-self.state[2]) < - np.pi):
	    self.state[2] = self.state[2]+2*np.pi
	#self.state[2] = self.normAngle(self.state[2])
        self.state = self.state + self.K.dot(arr - self.state)
        self.P = self.P-self.K.dot(self.P)
        
        
    def step_filter(self, v, omega, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
     
        if (imu_meas != None):
            self.prediction(v, omega, imu_meas)
        if z_t != None:
            if len(z_t[:][0:3]) > 0:
                print("saw tag", z_t[0][3])
                self.update(z_t[:][0:3])
        
        stArr = np.array(self.state)
        #stArr.shape=(3,1)
        return stArr
        
        

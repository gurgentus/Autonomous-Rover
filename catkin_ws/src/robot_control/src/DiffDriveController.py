#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        self.kp=0.5
        self.ka=1
        self.kb=0.2
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
 
    def normAngle(self, angle):
        newAngle = angle
        while newAngle <= -np.pi:
            newAngle = newAngle + 2*np.pi
        while newAngle > np.pi:
            newAngle = newAngle - 2*np.pi
        return newAngle

    
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """

        print(state)
        print(goal)
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        theta = state[2]
        rho = np.sqrt(dx**2+dy**2)
        #alpha = np.minimum(-theta + np.arctan2(dy,dx), - theta + np.arctan2(dy,dx)+2*np.pi)
        #beta = theta + alpha
        beta = np.arctan2(dy,dx)
        alpha = beta - theta
        print("beta", beta, "theta", theta, "alpha1", alpha)
	if (alpha > np.pi):
            alpha = alpha - 2*np.pi
    	else:
	    if (alpha < -np.pi):
            	alpha = alpha + 2*np.pi
        
        v = self.kp*rho
        omega = self.ka*alpha+self.kb*beta
        print(alpha)
        if (np.cos(alpha)*np.cos(alpha) < 0.2):
            v = 0.1
        else:
            v = np.cos(alpha)*rho

        if (v < 0):
            #exit()
	    v = 0.1
	   
        if (v > self.MAX_SPEED):
            v = self.MAX_SPEED
        
        omega = 4*alpha
        if (omega > self.MAX_OMEGA):
            omega = self.MAX_OMEGA
        #if (omega < -self.MAX_OMEGA):
        #    omega = -self.MAX_OMEGA

        done = False
        if (np.absolute(dx) < 0.01 and np.absolute(dy) < 0.01):
            done = True
        
	return (v,omega,done)


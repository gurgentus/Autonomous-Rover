#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files
from MyShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        self.kalman_filter = KalmanFilter(world_map)
        print("INITSTATE", self.kalman_filter.state)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.vel = 0
	self.omega = 0
        self.curInd = 0
        
        self.path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
        print(self.path)
        self.curGoal = self.path[0]
        self.done = False
    
    def process_measurements(self):
        """ 
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements()
        print("Mesurements", meas)
        imu_meas = self.ros_interface.get_imu()
        print(imu_meas)
        updatedPosition = self.kalman_filter.step_filter(self.vel, self.omega, imu_meas, meas)
        print(np.linalg.norm(self.curGoal - updatedPosition[0:1]))
        if ( (np.abs(self.curGoal[0] - updatedPosition[0]) > 0.1) or (np.abs(self.curGoal[1] - updatedPosition[1]) > 0.1) ):
            (v,omega,done) = self.diff_drive_controller.compute_vel(updatedPosition, self.curGoal)
            self.vel = v
	    self.omega = omega
            print("commanded vel:", v, omega)
            self.ros_interface.command_velocity(v, omega)
        else:
            print("updating")
            self.curInd = self.curInd + 1
            if self.curInd < len(self.path):
                self.curGoal = self.path[self.curInd]
            else:
                self.done = True

        updatedPosition.shape=(3,1)
        
        return
    
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    time = rospy.get_time()
    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    #while (rospy.get_time()-time < 1):
    while not rospy.is_shutdown() and not robotControl.done:
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass



#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)
obstacle_threshold = 1.9
mininum_steering_angle = -45
previous_steering_angle = 0
steering_angle_error = 0
increment = None
theta_gain = 1.7  # P gain used to get ref_vx (Inner controller of cascade controller)
pos_gain = 0.07   # P gain used to get ref_vy (Inner controller of Cascade Controller)
p_gain = 30.0     # Gain for the final controller controlling acceleration (Outer controller)

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def preprocess_lidar(ranges):
    """ Preprocess the LiDAR scan array and check if obstacle is detected or not.
    """
    proc_ranges = []
    for i in range(len(ranges)):
        if math.isnan(ranges[i]):
            continue
        elif ranges[i] > obstacle_threshold:
            proc_ranges.append(0) # Free space found
        else:
            proc_ranges.append(1) # Obstacle found
    return proc_ranges


def steering_angle_to_goal(proc_ranges, increment):	
    goal_steering_angle = 0

    # Checking laser scans and determine the angular position of the opening
    free_space = [ i for i, e in enumerate(proc_ranges) if e == 0 ]
    
    for i in free_space:
        goal_steering_angle += mininum_steering_angle + increment/2  + i*increment

    return goal_steering_angle/len(free_space) if len(free_space) > 1 else goal_steering_angle


def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    # Example of publishing acceleration command on velocity velCallback
    error_theta = previous_steering_angle

    #Acceleration desired along x and y based on the obstacles ahead
    ref_vx = theta_gain*np.exp(-0.02*np.abs(error_theta)*2) ###
    ref_vy = pos_gain*error_theta

    #Error from the desired linear velocity
    error_vx = ref_vx - msg.x
    error_vy = ref_vy - msg.y
    # Multiplying with another P gain for controlled motion
    pub_acc_cmd.publish(p_gain*error_vx,p_gain*error_vy,0)

def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range

    global increment
    global previous_steering_angle
    global steering_angle_error

    # Obtain the laser scans and preprocess them find the goal point in the LIDAR ranges array
    proc_ranges = preprocess_lidar(msg.ranges)
    increment = 10

    goal_steering_angle = steering_angle_to_goal(proc_ranges, increment)

    #Calculating the angle error 
    steering_angle_error = round((goal_steering_angle + previous_steering_angle)/2)

    # Updating the goal coordinates using the theta error
    free_space = [2*np.cos(np.deg2rad(steering_angle_error)) , 2*np.sin(np.deg2rad(steering_angle_error))]
    previous_steering_angle = np.rad2deg(np.arctan2(free_space[1],free_space[0]))
    

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
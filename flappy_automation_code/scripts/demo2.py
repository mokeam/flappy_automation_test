#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

goal = [0, 0]
theta_gain = 1.7  # P gain used to get ref_vx (Inner controller of cascade controller)
pos_gain = 0.07   # P gain used to get ref_vy (Inner controller of Cascade Controller)
p_gain = 30.0     # Gain for the final controller controlling acceleration (Outer controller)
min_ang = -45     # Minimum angle
ang_diff = 10     # Angle increments between each laserscan signal
scan = [0.0 for i in range(9)] # Defining scan variable

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for error_vxvelocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3,velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)
    # Ros spin to prevent program from exiting
    rospy.spin()

def angconv(x):
    # Converting an angle from radians to degrees
    return np.rad2deg(np.arctan2(x[1],x[0]))	

def velCallback(msg):
    # Calculating the error between 0 degrees where we desire our opening to be and where the actual opening is	
    error_theta = angconv(goal)

    #Acceleration desired along x and y based on the obstacles ahead
    ref_vx = theta_gain*np.exp(-0.02*np.abs(error_theta)*2) ###
    ref_vy = pos_gain*error_theta

    #Error from the desired linear velocity
    error_vx = ref_vx - msg.x
    error_vy = ref_vy - msg.y
    # Multiplying with another P gain for controlled motion
    pub_acc_cmd.publish(p_gain*error_vx,p_gain*error_vy,0)


def angle_calc():	
    goal_angle = 0

    # Checking laser scans and determine the angular position of the opening
    goals = [ i for i, e in enumerate(scan) \
                            if e == 0 ]
    
    for i in goals:
        goal_angle += min_ang+ang_diff/2 \
                + i*ang_diff

    return goal_angle/len(goals) if len(goals) > 1 else goal_angle


def laserScanCallback(msg):

    global goal, scan
    # Avoiding false opening while decision making when a laser scans between small gaps of rocks
    scan = [1 if i<=1.9 else 0 for i in msg.ranges]
    
    goal_angle = angle_calc()
    last_goal_angle = angconv(goal)

    #Calculating the angle error 
    error_theta = round((goal_angle + last_goal_angle)/2)

    # Updating the goal coordinates using the theta error
    goal = [2*np.cos(np.deg2rad(error_theta)) , 2*np.sin(np.deg2rad(error_theta))]
    

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
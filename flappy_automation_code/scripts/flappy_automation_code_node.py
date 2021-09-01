#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

#Parameters
bubble_radius = 12
scan_threshold = 6.0
gap_threshold = 1.9
increment = 0
minimum_steering_angle = -45
previous_goal_steering_angle = 0
current_goal_steering_angle = 0
K_x = 1.2 
K_y = 0.004
K_acc = 12

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    # Example of publishing acceleration command on velocity velCallback
    global current_goal_steering_angle
    global previous_goal_steering_angle
   
    #Calculating the angle error 
    error_theta = previous_goal_steering_angle

    #Acceleration desired along x and y based on the obstacles ahead
    R_vx = K_x*np.exp(-0.02*np.abs(error_theta)*2) ###
    R_vy = K_y*error_theta

    #Error from the des10ired linear velocity
    E_vx = R_vx - msg.x
    E_vy = R_vy - msg.y
    # Multiplying with another P gain for controlled motion
    pub_acc_cmd.publish(K_acc*E_vx,K_acc*E_vy,0)


def preprocess_lidar(ranges):
    """ Preprocess the LiDAR scan array.
    """
    global scan_threshold
    proc_ranges = []
    # print(ranges)
    for i in range(len(ranges)):
        if math.isnan(ranges[i]):
            proc_ranges.append(0)
        elif ranges[i] > scan_threshold:
            proc_ranges.append(scan_threshold)
        else:
            proc_ranges.append(ranges[i])

    return proc_ranges

def eliminate_bubble(proc_ranges, dist, index):
    """ Return the array of ranges after eliminating the points inside the bubble radius
    """
    global bubble_radius
    global increment
    angle = bubble_radius/dist
    start_idx = round(index - (angle/increment))
    end_idx = round(index + (angle/increment))

    if end_idx >= len(proc_ranges):
        end_idx = len(proc_ranges)-1

    if start_idx < 0:
        start_idx = 0

    for i in range(int(start_idx), int(end_idx)+1):
        proc_ranges[i] = 0

    return proc_ranges

def find_max_gap(free_space_ranges):
    """ Return the start index & end index of the max gap in free_space_ranges
    """
    max_start_idx = 0
    max_size = 0

    current_idx = 0
    current_start = 0

    # print(free_spac10e_ranges)

    while current_idx < len(free_space_ranges):

        current_size = 0
        current_start = current_idx

        while current_idx < len(free_space_ranges) and free_space_ranges[current_idx] > gap_threshold:
            current_size += 1
            current_idx += 1

        if current_size > max_size:
            max_start_idx = current_start
            max_size = current_size
            current_size = 0

        current_idx += 1

    if current_size > max_size:
        max_start_idx = current_start
        max_size = current_size

    return max_start_idx, max_start_idx+max_size-1

def find_best_point(start_idx, end_idx, ranges):
    """Start_i & end_i are start and end indicies of max-gap range, respectively
    Return index of best point in ranges
    """
    idx = (start_idx + end_idx)/2
    print("Best Point:"+str(ranges[idx]))
    return idx
    
def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range85
    # print("Laser range: {}, angle: {}".format(msg.ranges[0], msg.angle_min))
    
    global increment
    global current_goal_steering_angle
    global previous_goal_steering_angle

    # Obtain the laser scans and preprocess them Find the closest point in the LIDAR ranges array
    ranges = msg.ranges
    
    increment = np.rad2deg(msg.angle_increment)
    # print("Ranges: "+str(msg.angle_increment))

    proc_ranges = preprocess_lidar(ranges)
    # print("Processed Ranges"+str(prclosest_pointoc_ranges))

    #Find closest point to LiDAR
    closest_point = min(proc_ranges)
    # print("Closeset Point: "+str(closest_point))
    closest_index = proc_ranges.index(closest_point)

    #Eliminate all points inside 'bubble' (set them to zero)
    post_bubble = eliminate_bubble(proc_ranges, closest_point, closest_index)
    # print("Post Bubble: "+str(post_bubble))

    #Find max length gap
    start_idx, end_idx = find_max_gap(post_bubble)

    #Find the best point in the gap
    final_idx = find_best_point(start_idx, end_idx, post_bubble)
    # print("Best Point: "+str(final_idx))

    current_goal_steering_angle = (minimum_steering_angle + increment/2 )+ (final_idx *  increment)

    #Calculating the angle error 
    steering_angle_error = round((current_goal_steering_angle + previous_goal_steering_angle)/2)

    # Updating the goal coordinates using the theta error
    free_space = [2*np.cos(np.deg2rad(steering_angle_error)) , 2*np.sin(np.deg2rad(steering_angle_error))]

    previous_goal_steering_angle = np.rad2deg(np.arctan2(free_space[1],free_space[0]))

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
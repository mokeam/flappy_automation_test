#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

free_space = np.zeros(2)
asteroids = np.zeros(9)

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

    GAIN_BLEND = 3
    GAIN_FREESPACE_ANGLE = 0.05
    GAIN_VELCOITY = 20.0
    GATE_ANGLE = np.rad2deg(np.arctan2(free_space[1], free_space[0]))
    gatex = GAIN_BLEND * np.exp(-0.02 * np.abs(GATE_ANGLE) * 2)
    gatey = GAIN_FREESPACE_ANGLE * GATE_ANGLE
    x = GAIN_VELCOITY * (gatex - msg.x)
    y = GAIN_VELCOITY * (gatey - msg.y)
    pub_acc_cmd.publish(Vector3(x, y, 0))


def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
   
    global free_space
    global asteroids
    GATE = []
    FOV = []
    GATE_ANGLE = 0
    MAXIMUM_DISTANCE = 2
    CELL_THRESHOLD = 0.3
    FOV_ANGLE = 10
    FOV_MIN_ANGLE = -45
    LASER_FILTER = 0.45
    for i in msg.ranges:
        if i <= MAXIMUM_DISTANCE:
            FOV.append(1)
        else:
            FOV.append(0)
    for x, e in enumerate(asteroids):
        asteroids[x] = FOV[x] * LASER_FILTER + e * (1 - LASER_FILTER)
        if e <= CELL_THRESHOLD:
            GATE.append(x)

    for i in GATE:
        if len(GATE) > 1:
            GATE_ANGLE += (FOV_MIN_ANGLE + FOV_ANGLE / 2 + i * FOV_ANGLE) / len(GATE)
        else:
            GATE_ANGLE = FOV_MIN_ANGLE + FOV_ANGLE / 2 + i * FOV_ANGLE

    free_space = [2 * np.cos(np.deg2rad(GATE_ANGLE)), 2 * np.sin(np.deg2rad(GATE_ANGLE))]

    print("Laser range: {}, angle: {}".format(msg.ranges[0], msg.angle_min))

    # Obtain the laser scans and preprocess them Find the closest point in the LIDAR ranges array Draw a safety
    # bubble around the closest point and set all points inside this bubble to 0. All other non-zero points are now
    # considered gaps or free space. Find the maxlength "gap" in other words, the largest number of consecutive
    # non-zero elements in the ranges array Find the best goal point in this gap Actuate flappy bird to move towards
    # the free space


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass

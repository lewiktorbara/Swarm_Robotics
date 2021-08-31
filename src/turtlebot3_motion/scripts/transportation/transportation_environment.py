#! /usr/bin/env python

# file contains implementation of artificial environment for transporting the box algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from std_msgs.msg import *

import re
import csv
import math

# publisher
pub_ = None

# services
srv_robot_0_start_ = None
srv_robot_1_start_ = None

# values
robot_0_start_switch_ = '/robot_0_start_switch'
robot_1_start_switch_ = '/robot_1_start_switch'

# customizable variables
box_yaw_ = 0.0
desired_yaw_ = 0.0
robot0_yaw_ = 0
robot1_yaw_ = 0

angle_difference_ = math.pi / 30 # 6 degrees

change_ = False


time_ = 0
time_zero_ = 0
iterator_ = 10
times_ = []
desired_yaws_ = []
box_yaws_ = []
robot0_yaws_ = []
robot1_yaws_ = []
saved_ = False

# clock subscriber's function
def clbk_clock(msg):
    global time_
    time_ = float(msg.clock.secs) + float(msg.clock.nsecs)/1000000000

# box odometry subscriber's function
def clbk_box_odom(msg):
    global box_yaw_

    # yaw
    quanternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quanternion)
    box_yaw_ = euler[2]

# robot_0 odometry subscriber's function
def clbk_r0_odom(msg):
    global robot0_yaw_

    # yaw
    quanternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quanternion)
    robot0_yaw_ = euler[2]

# robot_1 odometry subscriber's function
def clbk_r1_odom(msg):
    global robot1_yaw_

    # yaw
    quanternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quanternion)
    robot1_yaw_ = euler[2]

# initialization of node
def init():
    global change_, time_zero_, times_, desired_yaws_, robot0_yaws_, robot1_yaws_, box_yaws_
    rospy.sleep(7.)

    resp = srv_robot_0_start_(True)
    resp = srv_robot_1_start_(True)
    time_zero_ = time_
    for i in range(20):
        times_.append(time_-time_zero_)
        desired_yaws_.append(desired_yaw_)
        box_yaws_.append(box_yaw_)
        robot0_yaws_.append(robot0_yaw_)
        robot1_yaws_.append(robot1_yaw_)
        rospy.sleep(.5)
    
    change_ = True

# change of desired yaw
def yaw_change():
    global desired_yaw_
    if desired_yaw_ < -math.pi / 2:
        pass
    elif math.fabs(box_yaw_ - desired_yaw_) < angle_difference_:
        desired_yaw_ -= math.pi / 30

# function that publishes the desired yaw
def publish():
    msg = Float32()
    msg.data = desired_yaw_
    pub_.publish(msg)

# function that saves data gathered during the simulation
def save():
    global saved_
    with open('/home/hairy/data/transport.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(times_)
        writer.writerow(desired_yaws_)
        writer.writerow(box_yaws_)
        writer.writerow(robot0_yaws_)
        writer.writerow(robot1_yaws_)
    saved_ = True
    print 'saved'


def main():
    global srv_robot_0_start_, srv_robot_1_start_, pub_, iterator_, times_, robot0_yaws_, robot1_yaws_, box_yaws_, desired_yaws_

    # node initialization
    rospy.init_node('transport_environment')

    # publishers
    pub_ = rospy.Publisher('/desired_yaw', Float32, queue_size=1)

    # subscribers
    sub_box = rospy.Subscriber('/box/odom', Odometry, clbk_box_odom)
    sub_r0 = rospy.Subscriber('/robot_0/odom', Odometry, clbk_r0_odom)
    sub_r1 = rospy.Subscriber('/robot_1/odom', Odometry, clbk_r1_odom)
    sub_clock = rospy.Subscriber('/clock', Clock, clbk_clock)

    rospy.wait_for_message('/clock', Clock)

    # services
    rospy.wait_for_service(robot_0_start_switch_)
    rospy.wait_for_service(robot_1_start_switch_)

    srv_robot_0_start_ = rospy.ServiceProxy(robot_0_start_switch_, SetBool)
    srv_robot_1_start_ = rospy.ServiceProxy(robot_1_start_switch_, SetBool)

    init()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        publish()
        # changing the box movement desired direction
        if change_ == True:
            yaw_change()

        # data gathering
        if iterator_ == 10:
            iterator_ = 0
            times_.append(time_-time_zero_)
            desired_yaws_.append(desired_yaw_)
            box_yaws_.append(box_yaw_)
            robot0_yaws_.append(robot0_yaw_)
            robot1_yaws_.append(robot1_yaw_)

        # end of simulation condition
        if time_ - time_zero_ > 50 and saved_ == False:
            save()
            
        iterator_ += 1
        rate.sleep()

if __name__ == "__main__":
    main()
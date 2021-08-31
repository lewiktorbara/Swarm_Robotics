#! /usr/bin/env python

# this file contains subalgorithm for group search algorithm - follow the wall

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from turtlebot3_motion.srv import *

import math

active_ = False

# publisher
pub_ = None

# robot unit parameters
num_ = int(rospy.get_param('num'))
cmd_vel_ = '/robot_' + str(num_) + '/cmd_vel'
laserscan_ = '/robot_' + str(num_) + '/base_scan'
follow_the_wall_switch_ = '/robot_' + str(num_) + '/follow_the_wall_switch'

# customizable variables
x_linear_speed_ = 0.3
follow_side_ = None # True -> left / False -> right

# laser readings
regions_ = {}

# states
state_ = 0

#service callback activation
def follow_the_wall_switch(req):
    global active_, follow_side_
    active_ = req.active
    follow_side_ = req.leader
    res = SetBool2Response()
    res.success = True
    return res

# laserscan subscriber's function
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right': min(min(msg.ranges[92:127]), 3.2),
        'fright': min(min(msg.ranges[128:163]), 3.2),
        'front': min(min(msg.ranges[164:199]), 3.2),
        'fleft': min(min(msg.ranges[200:235]), 3.2),
        'left': min(min(msg.ranges[236:271]), 3.2),
    }

    take_action()

# function that determines state
def take_action():
    global regions_, follow_side_
    regions = regions_

    state_description = ''

    d = 0.6

    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] <= d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] <= d:
        state_description = 'case 3 - fright'
        if follow_side_ == True:
            change_state(2)
        else:
            change_state(0)
    elif regions['front'] > d and regions['fleft'] <= d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        if follow_side_ == True:
            change_state(0)
        else:
            change_state(2)
    elif regions['front'] <= d and regions['fleft'] > d and regions['fright'] <= d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] <= d and regions['fleft'] <= d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] <= d and regions['fleft'] <= d and regions['fright'] <= d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] <= d and regions['fright'] <= d:
        state_description = 'case 8 - fleft and fright'
        change_state(2)
    else:
        state_description = 'unknown case'


# change state
def change_state(state):
    global state_
    state_ = state

# find wall
def find_wall():
    global pub_, follow_side_
    msg = Twist()
    msg.linear.x = 0.2
    if follow_side_ == True:
        msg.angular.z = -0.3
    else:
        msg.angular.z = 0.3
    pub_.publish(msg)

# turning
def turn():
    global pub_, follow_side_
    msg = Twist()
    msg.linear.x = 0.0
    if follow_side_ == True:
        msg.angular.z = 0.5
    else:
        msg.angular.z = -0.5
    pub_.publish(msg)

# going straight
def follow_the_wall():
    global pub_, x_linear_speed_
    msg = Twist()
    msg.linear.x = x_linear_speed_
    msg.angular.z = 0.0
    pub_.publish(msg)





def main():
    global pub_, cmd_vel_, laserscan_, follow_the_wall_switch_

    # node initialization
    rospy.init_node('follow_the_wall')

    # publishers
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1)

    # subscribers
    sub_laser = rospy.Subscriber(laserscan_, LaserScan, clbk_laser)

    # services
    srv_follow_the_wall = rospy.Service(follow_the_wall_switch_, SetBool2, follow_the_wall_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue

        else:
            # state 0 finding the wall
            if state_ == 0:
                find_wall()

            # state 1 turning
            elif state_ == 1:
                turn()

            # state 2 following the wall / going straight
            elif state_ == 2:
                follow_the_wall()

        rate.sleep()

if __name__ == "__main__":
    main()
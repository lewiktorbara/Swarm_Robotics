#! /usr/bin/env python

# That file contains implementation of alternating walk algorithm

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import *
import random

active_ = False

# publisher
pub_ = None

#robot unit parameters
num_ = int(rospy.get_param('num'))
cmd_vel_ = '/robot_' + str(num_) + '/cmd_vel'
odom_ = '/robot_' + str(num_) + '/odom'
laser_ = '/robot_' + str(num_) + '/base_scan'
random_walk_switch_ = '/robot_' + str(num_) + '/random_walk_switch'

# robot positions
position_ = Point()

# customizable variables
x_linear_speed_ = 0.3 #float(rospy.get_param('linear_speed'))
z_angular_speed_ = 0.0 #float(ropsy.get_param('angular speed'))
time_in_walk_const_ = 5 #float(rospy.get_param('walk_time'))
send_rate_ = 20

# laser variables
regions_ = {}

# state
state_ = 0 

# service switch that activates node
def random_walk_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# laserscan subscriber's funciton
def clbk_laser(msg):
    global regions_
    regions_ = {
        'sfright': min(min(msg.ranges[92:127]), 3.2),
        'fright': min(min(msg.ranges[128:163]), 3.2),
        'front': min(min(msg.ranges[164:199]), 3.2),
        'fleft': min(min(msg.ranges[200:235]), 3.2),
        'sfleft': min(min(msg.ranges[236:271]), 3.2),
    }

# odometry subscriber's funciton
def clbk_odom(msg):
    global position_
    position_ = msg.pose.pose.position

# change state
def change_state(state):
    global state_
    state_ = state

# function that random initial turn time
def init_random():
    return random.uniform(0, 10)

# function that random turn time
def random_time():
    return random.uniform(1, 3)
    
# moving forward
def go_straight():
    msg = Twist()
    msg.linear.x = x_linear_speed_
    msg.angular.z = z_angular_speed_
    pub_.publish(msg)

# turn
def turn():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 1.0
    pub_.publish(msg)


def main():
    global pub_, count_loop_, random_walk_switch_, time_in_walk_const_, regions_, random_walk_switch

    # node initialization
    rospy.init_node('random_walk')

    # publishers
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1)

    # subscribers
    sub_laser = rospy.Subscriber(laser_, LaserScan, clbk_laser)

    # services
    srv = rospy.Service(random_walk_switch_, SetBool, random_walk_switch)

    # initial assignment
    time_in_turn = init_random()
    time_in_walk = time_in_walk_const_
    decrease_time = 1.0/send_rate_


    rate = rospy.Rate(send_rate_)
    while not rospy.is_shutdown():
        if regions_ == None or not active_:
            rate.sleep()
            continue

        # truning
        if state_ == 0:
            if time_in_turn <= 0:
                    change_state(1)
            else:
                time_in_turn = time_in_turn - decrease_time
                turn()
        
        # going straight
        if state_ == 1:
            if time_in_walk <= 0 or min(regions_['front'], regions_['fleft'], regions_['fright']) < 0.45:
                time_in_turn = random_time()
                change_state(0)
                time_in_walk = time_in_walk_const_
            else:
                time_in_walk = time_in_walk - decrease_time
                go_straight()
        

        rate.sleep()

if __name__ == '__main__':
    main()
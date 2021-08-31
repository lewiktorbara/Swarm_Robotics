#! /usr/bin/env python

# this file contains implementation of resource aggregation algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from turtlebot3_motion.srv import *


import math


# publisher
pub_ = None

# services
srv_random_walk_ = None
srv_environment_ = None
srv_environment_robot_ = None

# robot unit parameters
robs_num_ = int(rospy.get_param('rbts_num'))
num_ = int(rospy.get_param('num'))
laserscan_ = '/robot_' +str(num_) + '/base_scan'
odom_ = '/robot_' + str(num_) + '/odom'
cmd_vel_ = '/robot_' + str(num_) + '/cmd_vel'
random_walk_switch_ = '/robot_' + str(num_) + '/random_walk_switch'
environment_switch_ = '/environment_switch'
environment_robot_switch_ = '/environment_robot_switch'

# robot position
position_ = Point()
position_.x = -100
position_.y = -100
yaw_ = 0

# laser readings
memory_ = []
laser_readings_ = [0,0,0,0,0,0,0,0,0,0,]
laser_err_ = 0.5

# values for 10 and less robots
resource_positions_ = {
    0 : (0.0, 0.0),
    1 : (1.5, 1.5),
    2 : (-1.5, -1.5),
    3 : (-1.5, 1.5),
    4 : (1.5, -1.5),
}

# states
state_ = None
state_dict_ = {
    0: 'random_walk',
    1: 'wait',
    2: 'found resource',
}


# odometry subscriber's function
def clbk_odom(msg):
    global position_
    global yaw_

    position_ = msg.pose.pose.position

    quanternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quanternion)
    yaw_ = euler[2]

# laserscan subscriber's function
def clbk_laser(msg):
    global laser_readings_
    laser_readings_ = [
        min(min(min(msg.ranges[0:18]), msg.ranges[344:359]), 3.2),
        min(min(msg.ranges[19:55]), 3.2),
        min(min(msg.ranges[56:91]), 3.2),
        min(min(msg.ranges[92:127]), 3.2),
        min(min(msg.ranges[128:163]), 3.2),
        min(min(msg.ranges[164:199]), 3.2),
        min(min(msg.ranges[200:235]), 3.2),
        min(min(msg.ranges[236:271]), 3.2),
        min(min(msg.ranges[272:307]), 3.2),
        min(min(msg.ranges[308:343]), 3.2)
    ]

# initialization function
def init():
    global resource_positions_
    if robs_num_ <=10:
        del_num = robs_num_/2 - 1
        i = 4
        while i > del_num: 
            resource_positions_.pop(i)
            i -= 1

# change state
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'robot_' + str(num_) + 'changed state to....' + state_dict_[state]
        state_ = state

    if state_ == 0:
        resp = srv_random_walk_(True)
    elif state_ == 1 or state == 2:
        resp = srv_random_walk_(False)

# function checking weather robot is close to resource
def resource_check():
    global position_, resource_positions_, state_ 
    
    for key, value in resource_positions_.items():
        if math.sqrt(pow(position_.x - value[0] , 2) + pow(position_.y - value[1] , 2)) < 0.6:
            a = GroupEnvRequest()
            a.x = position_.x
            a.y = position_.y
            a.id = key
            resp = srv_environment_(a)
            if resp.success == True:
                change_state(2)
            else:
                resource_positions_.pop(key)

# function checking the odometry of detected by laser sensors obstacle
def get_position():
    d = 3.2
    index = 0
    pi = math.pi

    for i in range(0,10):
        if laser_readings_[i] < d:
            d = laser_readings_[i]
            index = i
        
    ang_dep = (float(index)/5 - 1)*pi
    ang_indep = yaw_ + ang_dep
    if ang_dep < 0:
        ang_indep = ang_indep + 2*pi
    else:
        ang_indep = ang_indep % (2*pi)

    x,y = 0,0

    if (ang_indep > 0 and ang_indep < pi/2):
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep)
    elif (ang_indep > pi/2 and ang_indep < pi):
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep)
    elif (ang_indep > pi and ang_indep < pi*1.5):
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep + pi/2)
    elif (ang_indep > pi*1.5 and ang_indep < 2*pi):
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep)


    x1 = position_.x + x
    y1 = position_.y + y

    return x1, y1

# functions that checks if the detected obstacle is free robot
def check_robots():
    global memory_

    x,y = get_position()
    a = GroupEnvRequest()

    a.x = x
    a.y = y
    a.id = num_
    resp = srv_environment_robot_(a)

    if resp.success == True:
        change_state(2)
    else:
        p = Point()
        p.x = x
        p.y = y
        memory_.insert(0, p)
        if len(memory_) >= 5:
            memory_.pop()
        change_state(0)

# function that checks wether the obstacle with that odometry was already taken under consideration
def in_memory():
    global memory_
    x, y = get_position()
    for point in memory_:
        if  math.fabs(point.x - x) < laser_err_ and \
            math.fabs(point.y - y) < laser_err_:
            return False
    return True

# publishing values to unable movement
def no_move():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub_.publish(msg)

def main():
    global pub_, srv_random_walk_, srv_environment_, srv_environment_robot_

    # node initialization
    rospy.init_node('resource_aggregation')
    init()

    # publishers
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1) 

    # subscribers
    sub_odom = rospy.Subscriber(odom_, Odometry, clbk_odom)
    sub_laser = rospy.Subscriber(laserscan_, LaserScan, clbk_laser)

    # services
    rospy.wait_for_service(random_walk_switch_)
    rospy.wait_for_service(environment_switch_)
    rospy.wait_for_service(environment_robot_switch_)

    srv_random_walk_ = rospy.ServiceProxy(random_walk_switch_, SetBool)
    srv_environment_ = rospy.ServiceProxy(environment_switch_, GroupEnv)
    srv_environment_robot_ = rospy.ServiceProxy(environment_robot_switch_, GroupEnv)


    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        # state 0 random walk
        if state_ == 0:
            if min(laser_readings_) < 0.5 and min(laser_readings_) > 0.15  and in_memory():
                change_state(1)
            resource_check()

        # state 1 wait
        elif state_ == 1:
            no_move()
            if min(laser_readings_) > 0.5 or min(laser_readings_) < 0.15:
                change_state(0)
            elif min(laser_readings_) < 0.5:
                check_robots()

        # state 2 resource found
        elif state_ == 2:
            no_move()

        rate.sleep()

if __name__ == "__main__":
    main()
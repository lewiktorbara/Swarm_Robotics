#! /usr/bin/env python

# this file contains implementation of group aggregation algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from turtlebot3_motion.msg import positions
from turtlebot3_motion.srv import *

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import math

# services
srv_random_walk_ = None
srv_wait_ = None
srv_environment_ = None

# robot unit parameters
num_ = int(rospy.get_param('num'))
laserscan_ = '/robot_' + str(num_) + '/base_scan'
odom_ = '/robot_' + str(num_) + '/odom'
random_walk_switch_ = '/robot_' + str(num_) + '/random_walk_switch'
wait_switch_ = '/robot_' + str(num_) + '/wait'

# robot position
position_ = Point()
yaw_ = 0

# laser readings
memory_ = []
laser_readings_ = [0,0,0,0,0,0,0,0,0,0,]
laser_err_ = 0.5

# states
status_ = 0
state_ = 0
state_dict_ = {
    0: 'wait',
    1: 'random_walk',
    2: 'aggregate',
}

# status switch
def status_switch(req):
    global status_ 
    status_ = 1
    change_state(2)
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# odometry subscriber's function
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
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

# function checking the odometry of detected by laser sensors obstacle
def get_position():
    d = 3.2
    index = 0
    pi = math.pi
    for i in range(0, 10):
        if laser_readings_[i] < d:
            d = laser_readings_[i]
            index = i
    
    ang_dep = (float(index)/5 - 1)*pi 
    ang_indep = yaw_ + ang_dep
    if ang_indep < 0:
        ang_indep = ang_indep + 2*pi
    else:
        ang_indep = ang_indep % (2*pi)
    
    x,y = 0,0
    if (ang_indep >0 and ang_indep < pi/2): 
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep)
    elif ang_indep > pi/2 and ang_indep < pi:
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep)
    elif ang_indep > pi and ang_indep < pi*1.5:
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep + pi/2)
    elif ang_indep > pi*1.5 and ang_indep < 2*pi:
        y = d * math.sin(ang_indep)
        x = d * math.cos(ang_indep) 
    
    x1 = position_.x + x
    y1 = position_.y + y
    
    return x1, y1

# functions that checks if the detected obstacle is free robot
def check_robots():
    global status_, memory_
    x, y = get_position()
    a = GroupEnvRequest()
    a.x = x
    a.y = y
    a.id = num_
    resp = srv_environment_(a)
    if resp.success == True:
        status_ = 1
        change_state(2)
    else:
        if status_ != 1:
            p = Point()
            p.x = x
            p.y = y
            memory_.insert(0, p)
            if len(memory_) >= 5:
                memory_.pop()
            change_state(1)

# change state 
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'robot_' + str(num_) + ' changed state to..... ' + state_dict_[state]
        state_ = state
    if state_ == 0 or state_ == 2:
        resp = srv_wait_(True)
        resp = srv_random_walk_(False)
    elif state_ == 1:
        resp = srv_wait_(False)
        resp = srv_random_walk_(True)

# function that checks wether the obstacle with that odometry was already taken under consideration
def in_memory():
    global memory_
    x, y = get_position()
    for point in memory_:
        if  math.fabs(point.x - x) < laser_err_ and \
            math.fabs(point.y - y) < laser_err_:
            return False
    return True

def main():
    global srv_random_walk_, srv_wait_, srv_environment_, laser_readings_

    # node initialization
    rospy.init_node(str(num_) + 'group_aggregation')


    # subscribers
    sub_laser = rospy.Subscriber(laserscan_, LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber(odom_, Odometry, clbk_odom)


    # services
    srv_status = rospy.Service('/' + str(num_) + 'status', SetBool, status_switch)

    rospy.wait_for_service(random_walk_switch_)
    rospy.wait_for_service(wait_switch_)
    rospy.wait_for_service('/environment_switch')
    
    srv_random_walk_ = rospy.ServiceProxy(random_walk_switch_, SetBool)
    srv_wait_ = rospy.ServiceProxy(wait_switch_, SetBool)
    srv_environment_ = rospy.ServiceProxy('/environment_switch', GroupEnv)


    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if laser_readings_ == None:
            rate.sleep()
            continue
        
        # state 0 random walk
        if state_ == 0:
            if status_ == 1:
                change_state(2)
            elif min(laser_readings_) > 0.5 or min(laser_readings_) < 0.15:
                change_state(1)
            elif min(laser_readings_) < 0.5:
                check_robots()

        # state 1 wait
        elif state_ == 1:
            if status_ == 1:
                change_state(2)
            elif min(laser_readings_) < 0.5 and min(laser_readings_) > 0.15 and in_memory():
                change_state(0)

        # state 2 aggregation
        elif state_ == 2:
            pass
        
        rate.sleep()
        
if __name__ == "__main__":
    main()
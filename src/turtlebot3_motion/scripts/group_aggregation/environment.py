#! /usr/bin/env python

# file contains implementation of artificial environment for group aggregation algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from turtlebot3_motion.msg import positions, Vector4
from turtlebot3_motion.srv import *

import re
import csv
import math

# services
srv_sim_ = None

# parameters
robs_num_ = int(rospy.get_param('rbts_num'))
subscribers_ = []
times_ = []
odom_ = {}
statuses_ = {}
time_ = 0.0
time_zero_ = 0.0 
laser_err_ = 0.5
saved_ = False

# switch that informs robot if found obstacle is free robot
def env_switch(req):
    global odom_, statuses_, laser_err_, time_, time_zero_, times_
    res = GroupEnvResponse()
    index = None
    for key, value in odom_.items(): 
        if  math.fabs(value.x - req.x) < laser_err_ and \
            math.fabs(value.y - req.y) < laser_err_ and \
            key != req.id:
            
            index = key
            break

    if index == None or statuses_[index] == 1:
        res.success = False
    else:
        res.success = True
        statuses_[index] = 1
        statuses_[req.id] = 1
        rospy.wait_for_service('/' + str(index) + 'status')
        srv = rospy.ServiceProxy('/' + str(index) + 'status', SetBool)
        resp = srv(True)
        times_.append(time_-time_zero_)
    return res

# function that initializes resources' positions and time values
def init():
    global odom_, statuses_, time_zero_
    time_zero_ = time_
    a = Vector4()
    for i in range(0, robs_num_):
        odom_[i] = a
        statuses_[i] = 0

# odometry subscriber's funciton
def clbk_odom(msg):
    global odom_, statuses_
    index = int(re.search(r'\d+', msg.header.frame_id).group())
    pos = Vector4()
    pos.x = msg.pose.pose.position.x
    pos.y = msg.pose.pose.position.y
    pos.z = 0.0
    pos.status = statuses_[index]
    odom_[index] = pos

# clock subscriber's funciton
def clbk_clock(msg):
    global time_
    time_ = float(msg.clock.secs) + float(msg.clock.nsecs)/1000000000
    
# function that saves gathered data to the file
def save():
    global times_, saved_
    while len(times_) < robs_num_/2:
        times_.append(500.0)
    with open('/home/hairy/data/test_ga.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow(times_)
    saved_ = True
    print times_
    print 'done'


def main():
    global saved_, srv_sim_
    
    # node initialization
    rospy.init_node('environment')

    # subscribers
    sub_clock = rospy.Subscriber('/clock', Clock, clbk_clock)
    rospy.wait_for_message('/clock', Clock)

    init()

    for i in range(0,robs_num_):
        sub_odom = rospy.Subscriber('/robot_' + str(i) + '/odom', Odometry, clbk_odom)
        subscribers_.append(sub_odom)

    # services
    srv = rospy.Service('/environment_switch', GroupEnv, env_switch)
    
    srv_sim_ = rospy.ServiceProxy('/simulation', SetBool)


    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        # end of simulation condition 
        if (len(times_) == robs_num_/2 or (time_ - time_zero_) > 500) and saved_ == False:
            save()


        rate.sleep()

if __name__ == '__main__':
    main()

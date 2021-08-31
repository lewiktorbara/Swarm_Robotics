#! /usr/bin/env python

# file contains implementation of artificial environment for resource aggregation algorithm

import rospy
from geometry_msgs.msg import Twist, Point
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from turtlebot3_motion.srv import *

import math
import csv

# services
srv_sim_ = None

# parameters
robs_num_ = int(rospy.get_param('rbts_num'))
times_ = []
times_single_ = []
time_ = 0.0
time_zero_ = 0.0
saved_ = False

# values for 10 and less robots
resource_positions_ = {
    0 : [0.0, 0.0, -100, 0],
    1 : [1.5, 1.5, -100, 0],
    2 : [-1.5, -1.5, -100, 0],
    3 : [-1.5, 1.5, -100, 0],
    4 : [1.5, -1.5, -100, 0],
}


# function that informs robot if the found resource is free, partially occupied or fully occupied
def environment_switch(req):
    global resource_positions_, times_, times_single_, time_, time_zero_
    res = GroupEnvResponse()

    if req.id in resource_positions_.keys():
        res.success = True
        if resource_positions_[req.id][2] != -100:
            resource_positions_.pop(req.id)
            times_.append(time_ - time_zero_)
        else:
            resource_positions_[req.id][2] = req.x
            resource_positions_[req.id][3] = req.y
            print resource_positions_[req.id][2:]
            times_single_.append(time_ - time_zero_)
    else:
        res.success = False

    return res 

# function that informs robot if found obstacle is robot that found the resource and wait for another robot
def environment_robot_switch(req):
    global resource_positions_, times_, time_, time_zero_

    res = GroupEnvResponse()

    for key, value in resource_positions_.items():
        if value[2] != -100:
            if math.fabs(value[2] - req.x) < 0.5 and math.fabs(value[3] - req.y) < 0.5:
                resource_positions_.pop(key)
                times_.append(time_ - time_zero_)
                res.success = True
                return res
    
    res.success = False
    return res

# function that initializes resources' positions and time values
def init():
    global time_zero_, resource_positions_
    time_zero_ = time_

    if robs_num_ <= 10:
        del_num = robs_num_/2 - 1
        i = 4
        while i > del_num: 
            resource_positions_.pop(i)
            i -= 1
    else:
        raise ValueError('Wrong number of robots')

# clock subscriber's function
def clbk_clock(msg):
    global time_
    time_ = float(msg.clock.secs) + float(msg.clock.nsecs)/1000000000

# function that saves gathered data to the files
def save():
    global times_, times_single_, saved_
    
    while len(times_) < robs_num_/2:
        times_.append(500.0)
    while len(times_single_) < robs_num_/2:
        times_single_.append(500.0)
    
    with open('/home/hairy/data/test_res.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow(times_)   
    
    with open('/home/hairy/data/test_res_single.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow(times_single_)  

    saved_ = True
    print times_
    print 'environment done'
    resp = srv_sim_(True)

def main():
    global saved_, srv_sim_

    # node initialization
    rospy.init_node('env')

    # subscribers
    sub_clock = rospy.Subscriber('/clock', Clock, clbk_clock)
    rospy.wait_for_message('/clock', Clock)

    init()

    # services
    srv_environment = rospy.Service('environment_switch', GroupEnv, environment_switch)
    srv_environment_robot = rospy.Service('environment_robot_switch', GroupEnv, environment_robot_switch)

    srv_sim_ = rospy.ServiceProxy('/simulation', SetBool)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        # end of simulation condition
        if (len(times_) == robs_num_/2 or (time_ - time_zero_) > 500) and saved_ == False:
            save()

        rate.sleep()

if __name__ == '__main__':
    main()
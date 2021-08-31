#! /usr/bin/env python

# this file contains implementation of transporting the box algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from std_msgs.msg import *

import math


# publisher
pub_ = None

# robot unit parameters
num_ = int(rospy.get_param('num')) # if 0 -> right
laserscan_ = '/robot_' + str(num_) + '/scan'
cmd_vel_ = '/robot_' + str(num_) + '/cmd_vel'
odom_ = '/robot_' + str(num_) + '/odom'
box_odom_ = '/box/odom'
robot_start_switch_ = '/robot_' + str(num_) + '_start_switch'

# robot position
position_ = Point()
position_.x = 0
position_.y = 0
yaw_ = 0
box_yaw_ = 0
desired_yaw_ = 0

# customizable variables
x_linear_speed_ = 0.15
initial_x_linear_speed_ = 0.15
max_dist_ = 0.2
ahead_ = True
alfa_ = 0
beta_ = 0
tau_ = 1.0
b_ = 1.0
max_angle_ = math.pi / 30 # 10 degrees
connects_  = True

# laser readings
laser_readings_ = [3.2]

# states
state_ = None
state_dict_ = {
    0: "Maintaining box direction",
    1: "Maintaining pusher direction",
    2: "Maintaining box velocity along pushong direction",
    3: "Aligning pushers symmetrically with respect to the box edges"
}

# switch that initialize the state
def start_switch(req):
    change_state(2)

    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# laser readings subscriber's funciton
def clbk_laser(msg):
    global laser_readings_

    laser_readings_ = msg.ranges[315:359] + msg.ranges[0:45]
    laser_readings_ = list(laser_readings_)
    
# odometry subscriber's funciton
def clbk_odom(msg):
    global position_
    global yaw_, beta_

    position_ = msg.pose.pose.position
    # yaw
    quanternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quanternion)
    yaw_ = euler[2]
    beta_ = yaw_ - box_yaw_

# box's odometry subscriber's function
def clbk_box_odom(msg):
    global box_yaw_, alfa_

    # yaw
    quanternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quanternion)
    box_yaw_ = euler[2]

# desired yaw subscriber's function
def clbk_desired_yaw(msg):
    global desired_yaw_, alfa_

    desired_yaw_ = msg.data
    alfa_ = box_yaw_ - desired_yaw_

# returns distance to the cornder and angle of it
def scan_corner():
    dist = min(laser_readings_)
    eps = laser_readings_.index(dist)
    
    if num_ == 0:
        eps = eps - 26
    else:
        eps = eps - 66

    eps = eps * math.pi / 180

    return dist/2, eps

# change state
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'robot_' + str(num_) + 'changed state to....' + state_dict_[state]
        state_ = state

# initial function
def init():
    global pub_, laser_readings_
    
    if min(laser_readings_) < 0.2:
        stay()
    else:
        initial_go_forward()

# going forward at the begninng
def initial_go_forward():
    global pub_
    msg = Twist()
    msg.linear.x = initial_x_linear_speed_
    msg.angular.z = 0.0
    pub_.publish(msg)

# going forward
def go_forward():
    global pub_
    msg = Twist()
    msg.linear.x = x_linear_speed_
    msg.angular.z = 0.0
    pub_.publish(msg)

# no move
def stay():
    global pub_
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub_.publish(msg)

# going forward with specified velocity
def control_fwd():
    global pub_
    
    vel = tau_*b_*math.fabs(math.sin(alfa_))

    msg = Twist()
    msg.linear.x = min(vel, x_linear_speed_)
    msg.angular.z = 0.0

    pub_.publish(msg)

# rotate
def turn():
    global pub_
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = -tau_*(alfa_+beta_)
    pub_.publish(msg)

# finding corner
def find_corner(val):
    global pub_
    msg = Twist()
    msg.linear.x = tau_*val[0]
    msg.angular.z = tau_*val[1]
    pub_.publish(msg)

# checks whether robot is in forward or not
def is_ahead():
    if (alfa_ > 0 and num_ == 0) or (alfa_ < 0 and num_ == 1):
        return True
    else:
        return False

def main():
    global pub_, state_

    # node initialization
    rospy.init_node('transport')

    # publishers
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1)

    # subscribers
    sub_laser = rospy.Subscriber(laserscan_, LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber(odom_, Odometry, clbk_odom)
    sub_box_odom = rospy.Subscriber(box_odom_, Odometry, clbk_box_odom)
    sub_desired_box_yaw = rospy.Subscriber('/desired_yaw', Float32, clbk_desired_yaw)

    # services
    srv_start = rospy.Service(robot_start_switch_, SetBool, start_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if state_ == None:
            init()

        # maintaining box direction
        elif state_ == 0:
            if min(laser_readings_[32:60]) > max_dist_:
                change_state(3)
            elif math.fabs(alfa_) < max_angle_ / 3:
                change_state(2)
            else:
                if is_ahead():
                    stay()
                else:
                    control_fwd()

        # maintaining pusher direction
        elif state_ == 1:
            if min(laser_readings_[32:60]) > max_dist_:
                change_state(3)
            elif math.fabs(yaw_ - desired_yaw_) < max_angle_ / 3:
                if math.fabs(alfa_) > max_angle_:
                    change_state(0)
                else:
                    change_state(2)
            else:
                turn()
            
        # maintaining box velocity
        elif state_ == 2:
            if min(laser_readings_[32:60]) > max_dist_:
                change_state(3)
            elif math.fabs(yaw_ - desired_yaw_) > max_angle_:
                change_state(1)
            elif math.fabs(alfa_) > max_angle_:
                change_state(0)
            else:
                go_forward()

        # aligning pushers to the box
        elif state_ == 3:
            if min(laser_readings_[42:50]) < 0.15:
                change_state(0)
            else:
                find_corner(scan_corner())

        rate.sleep()

if __name__ == "__main__":
    main()
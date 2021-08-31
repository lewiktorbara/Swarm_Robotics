#! /usr/bin/env python

# this file contains subalgorithm for group search algorithm - go to the point

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
odom_ = '/robot_' + str(num_) + '/odom'
laserscan_ = '/robot_' + str(num_) + '/base_scan'
go_to_point_switch_ = '/robot_' + str(num_) + '/go_to_point_switch'

# neighbour robot parameters
neighbour_num_ = int(rospy.get_param('n_num'))
neighbour_odom_ = '/robot_' + str(neighbour_num_) + '/odom'

# robot position
position_ = Point()
yaw_ = 0
# neighbour robot position
neighbour_position_ = Point()

# goal
desired_position_ = Point()
desired_position_.x = float(rospy.get_param('des_pos_x'))
desired_position_.y = float(rospy.get_param('des_pos_y'))
desired_position_.z = 0.0

# parameters
yaw_precision_ = math.pi/90 # +/- 2 degree allowed
dist_precision_ = 0.1
leader_ = None

# customizable variables
x_linear_speed_ = 0.3

# states
state_ = 0

# service callback activation
def go_to_point_switch(req):
    global active_, leader_
    active_ = req.active
    leader_ = req.leader
    res = SetBool2Response()
    res.success = True
    return res

# odometry subscriber's function 
def clbk_odom(msg):
    global position_, yaw_

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

# neighbour odometry subscriber's function
def clbk_nodom(msg):
    global neighbour_position_, desired_position_


    if leader_ == False:
        # position
        neighbour_position_ = msg.pose.pose.position

        # yaw
        quanternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quanternion)
        neighbour_yaw_ = euler[2]

        neighbour_position_.x = neighbour_position_.x + math.cos(neighbour_yaw_ + math.pi/2)
        neighbour_position_.y = neighbour_position_.y + math.sin(neighbour_yaw_ + math.pi/2)

        if math.fabs(neighbour_position_.x - desired_position_.x) > 0.1 or math.fabs(neighbour_position_.y  - desired_position_.y) > 0.1 or state_ == 2:
            desired_position_ = neighbour_position_
            change_state(0)
        


# change state
def change_state(state):
    global state_
    state_ = state

# turning the robot to point the direction of destination
def fix_yaw():
    global yaw_, yaw_precision_, desired_position_, position_, pub_, leader_
    

    desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = desired_yaw - yaw_
    
    msg = Twist()

    if math.fabs(err_yaw) > yaw_precision_:
        cap = yaw_ + math.pi
        z = 0
        if math.fabs(err_yaw) > yaw_precision_*5:
            z = 0.7
        else:
            z = 0.2

        if cap > math.pi:
            cap -= 2*math.pi
            if desired_yaw < yaw_ and desired_yaw > cap:
                msg.angular.z = -z
            else:
                msg.angular.z = z
        else:
            if desired_yaw > yaw_ and desired_yaw < cap:
                msg.angular.z = z
            else:
                msg.angular.z = -z
            

        pub_.publish(msg)

    else:
        change_state(1)

# going straight
def go_straight():
    global yaw_, yaw_precision_, desired_position_, position_, dist_precision_, x_linear_speed_, pub_
    
    desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = desired_yaw - yaw_

    if math.fabs(err_yaw) > 2*yaw_precision_:
        change_state(0)

    else:
        err_pos = math.sqrt(pow(desired_position_.x - position_.x , 2) + pow(desired_position_.y - position_.y , 2))
        if err_pos > dist_precision_:
            msg = Twist()
            msg.linear.x = x_linear_speed_
            pub_.publish(msg)
        else:
            change_state(2)

# already close to destination point
def done():
    global pub_
    if math.sqrt(pow(desired_position_.x - position_.x , 2) + pow(desired_position_.y - position_.y , 2)) > dist_precision_:
        change_state(0)
    else:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub_.publish(msg)

def main():
    global pub_, cmd_vel_, odom_, neighbour_odom_, go_to_point_switch_

    # node initialization
    rospy.init_node(str(num_) + 'go_to_point_flock')

    # publishers
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1)

    # subscribers
    sub_odom = rospy.Subscriber(odom_, Odometry, clbk_odom)
    sub_nodom = rospy.Subscriber(neighbour_odom_, Odometry, clbk_nodom)

    # services
    srv_go_to_point = rospy.Service(go_to_point_switch_, SetBool2, go_to_point_switch)



    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            # state 0 fixing the yaw
            if state_ == 0:
                fix_yaw()

            # state 1 going straight    
            elif state_ == 1:
                go_straight()

            # state 2 already close to destination
            elif state_ == 2:
                done()

        rate.sleep()

if __name__ == "__main__":
    main()
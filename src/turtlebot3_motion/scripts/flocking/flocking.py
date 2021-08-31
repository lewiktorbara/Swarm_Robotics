#! /usr/bin/env python

# this file contains implementation of group search algorithm

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from turtlebot3_motion.srv import *
from turtlebot3_motion.msg import Vector4

import math
import random

# publisher
pub_ = None

# services
srv_leadership_ = None
srv_go_to_point_ = None
srv_follow_the_wall_ = None
srv_random_walk_ = None
srv_status_change_ = None
srv_environment_ = None

# robot unit parameters
robs_num_ = int(rospy.get_param('rbts_num'))
num_ = int(rospy.get_param('num'))
cmd_vel_ = '/robot_' + str(num_) + '/cmd_vel'
laserscan_ = '/robot_' + str(num_) + '/base_scan'
odom_ = '/robot_' + str(num_) + '/odom'
leadership_switch_ = '/robot_' + str(num_) + '/leadership_switch'
go_to_point_switch_ = '/robot_' + str(num_) + '/go_to_point_switch'
follow_the_wall_switch_ = '/robot_' + str(num_) + '/follow_the_wall_switch'
random_walk_switch_ = '/robot_' + str(num_) + '/random_walk_switch'
status_change_switch_ = '/robot_' + str(num_) + '/status_change_switch'
environment_switch_ = '/environment_switch'

# neighbour robot parameters
neighbour_num_ = int(rospy.get_param('n_num'))
neighbour_odom_ = '/robot_' + str(neighbour_num_) + '/odom'
neighbour_leadership_switch_ = '/robot_' + str(neighbour_num_) + '/leadership_switch'
neighbour_status_change_switch_ = '/robot_' + str(neighbour_num_) + '/status_change_switch'

# robot position
position_ = Point()
yaw_ = 0
# neighbour robot position
neighbour_position_ = Point()
neighbour_yaw_ = 0
shifted_nposition_ = Point()

# customizable variables
initial_turn_time_ = 0
initial_turn_timeout_ = 40
initial_turn_rand_ = 0

# goal
desired_position_ = Point()
desired_position_.x = float(rospy.get_param('des_pos_x'))
desired_position_.y = float(rospy.get_param('des_pos_y'))
desired_position_.z = 0.0
shifted_position_ = Point()

# laser readings
laser_readings_ = {}

# states
status_ = 0
state_ = 0
state_dict_ = {
    0: 'leadership_estimation',
    1: 'go_to_point',
    2: 'follow the wall',
    3: 'wait',
    4: 'random walk',
    5: 'done',
    6: 'initial turn'
}

leader_ = None
follow_side_ = True

# values for 10 and less robots
resource_positions_ = {
    0 : [0.0, 0.0, -100, 0],
    1 : [1.5, 1.5, -100, 0],
    2 : [-1.5, -1.5, -100, 0],
    3 : [-1.5, 1.5, -100, 0],
    4 : [1.5, -1.5, -100, 0]
}

wait_time_treshhold_ = 0
minimal_rw_time_ = 0

# status change switch
def status_change_switch(req):
    global status_
    status_ = 1
    change_state(5)
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done'
    return res

# laserscan subscribers's function
def clbk_laser(msg):
    global laser_readings_
    laser_readings_ = {
        'right': min(min(msg.ranges[92:127]), 3.2),
        'fright': min(min(msg.ranges[128:163]), 3.2),
        'front': min(min(msg.ranges[164:199]), 3.2),
        'fleft': min(min(msg.ranges[200:235]), 3.2),
        'left': min(min(msg.ranges[236:271]), 3.2),
    }

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

    shifted_position_.x = position_.x + math.cos(yaw_ + math.pi/2)
    shifted_position_.y = position_.y + math.sin(yaw_ + math.pi/2)

# neighbour odometry subscriber's function
def clbk_nodom(msg):
    global neighbour_position_, neighbour_yaw_, shifted_nposition_

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

    shifted_nposition_.x = neighbour_position_.x + math.cos(neighbour_yaw_ + math.pi/2)
    shifted_nposition_.y = neighbour_position_.y + math.sin(neighbour_yaw_ + math.pi/2)

# initialization of node
def init():
    global resource_positions_
    
    if robs_num_ <=10:
        del_num = robs_num_/2 -1
        i = 4
        while i > del_num:
            resource_positions_.pop(i)
            i -= 1

# function that changes states and activates corresponding to them scripts
def change_state(state):
    global state_, state_dict_, leader_, follow_side_
    if state is not state_:
        print 'robot_' +str(num_) + ' changed state to....' + state_dict_[state]
        state_ = state
    gtp = SetBool2Request()
    fw = SetBool2Request()
    rw =SetBoolRequest()
    if leader_ == None:
        gtp.leader = True
    else:
        gtp.leader = leader_
    fw.leader = follow_side_

    if state_ == 0 or state_ == 3 or state_ == 5 or state_ == 6:
        gtp.active = False
        fw.active = False
        rw.data = False
    elif state_ == 1:
        gtp.active = True
        fw.active = False
        rw.data = False
    elif state_ == 2:
        gtp.active = False
        fw.active = True
        rw.data = False
    elif state_ == 4:
        gtp.active = False
        fw.active = False
        rw.data = True

    resp = srv_go_to_point_(gtp)
    resp = srv_follow_the_wall_(fw)
    resp = srv_random_walk_(rw)

# the function that estimates the leadership in the pair
def leadership_estimation():
    global leader_, yaw_, neighbour_yaw_, srv_leadership_
    if leader_ == None:
        if yaw_ > neighbour_yaw_:
            resp = srv_leadership_(False)
            leader_ = True
        elif yaw_ < neighbour_yaw_:
            resp = srv_leadership_(True) 
            leader_ = False
        else:
            random_turn()
    elif leader_ == True:    
        change_state(6)
    else:
        change_state(1)

# turns robots by random angle
def random_turn():
    global pub_
    z = random.uniform(-1,1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = z
    pub_.publish(msg)

# function that makes leader wait for following him robot
def wait():
    global minimal_rw_time_
    if dist_calc() < 0.5 or wait_time_treshhold_ <= 0:
        minimal_rw_time_ = 60
        change_state(4)
    else:
        global pub_
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub_.publish(msg)
    
# function that estimates the distance between paired robots
def dist_calc():
    return math.sqrt((shifted_position_.x - neighbour_position_.x)**2 + (shifted_position_.y - neighbour_position_.y)**2 )

# service switch that sets the leadership in pair
def leadership_switch(req):
    global leader_
    leader_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# function that estimates wether follow the wall state can be left
def leave_ftw():
    global leader_, desired_position_, position_, neighbour_position_, yaw_, laser_readings_
    destination = None
    if leader_:
        destination = desired_position_
    else:
        destination = shifted_nposition_

    desired_yaw = math.atan2(destination.y - position_.y, destination.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    ferr_yaw = math.fabs(err_yaw)
    if (ferr_yaw <= math.pi / 10 and laser_readings_['front'] < 1) or\
        (err_yaw > 0 and ferr_yaw > math.pi / 10 and ferr_yaw <= math.pi *3 / 10 and laser_readings_['fleft'] < 1 ) or\
        (err_yaw > 0 and ferr_yaw > math.pi * 3 / 10 and ferr_yaw <= (math.pi / 2) and laser_readings_['left'] < 1) or\
        (err_yaw < 0 and ferr_yaw > math.pi / 10 and ferr_yaw <= math.pi *3 / 10 and laser_readings_['fright'] < 1) or\
        (err_yaw < 0 and ferr_yaw > math.pi *3 / 10 and ferr_yaw <= (math.pi / 2) and laser_readings_['right'] < 1) or\
        (min(min(laser_readings_['front'], laser_readings_['fright']), laser_readings_['fleft']) < 0.6 and ferr_yaw < math.pi / 2 ):
        pass
    else:
        if leader_ == True:
            change_state(4)
        else: 
            change_state(1)

# normalizes angle to set (-pi, pi)   
def normalize_angle(angle):
    if (math.fabs(angle) > math.pi):
        angle = angle - ((2 * math.pi * angle) / math.fabs(angle))
    return angle

# checking weather the robot is close to the resource and informing the environment
def resource_check():
    global position_, resource_positions_, status_
    for key, value in resource_positions_.items():
        if math.sqrt(pow(position_.x - value[0] , 2) + pow(position_.y - value[1] , 2)) < 0.6:
            a = GroupEnvRequest()
            a.id = key
            resp = srv_environment_(a)
            if resp.success == True:
                status_ = 1
                resp2 = srv_status_change_(True)
                change_state(5)
                return True
            else:
                resource_positions_.pop(key)
                return False
    return False

def main():
    global laserscan_, odom_, neighbour_odom_, pub_, cmd_vel_, wait_time_treshhold_, minimal_rw_time_, follow_side_, initial_turn_rand_, initial_turn_time_
    global srv_leadership_, leadership_switch_, neighbour_leadership_switch_
    global srv_go_to_point_, go_to_point_switch_, srv_follow_the_wall_, follow_the_wall_switch_, srv_random_walk_, srv_status_change_, srv_environment_

    # node initialization
    rospy.init_node(str(num_) + 'flocking')

    init()

    # publisher
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1)

    # subscribers
    sub_laser = rospy.Subscriber(laserscan_, LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber(odom_, Odometry, clbk_odom)
    sub_nodom = rospy.Subscriber(neighbour_odom_, Odometry, clbk_nodom)

    # services
    srv_lead = rospy.Service(leadership_switch_, SetBool, leadership_switch)
    srv_status_change = rospy.Service(status_change_switch_, SetBool, status_change_switch)
    
    rospy.wait_for_service(neighbour_leadership_switch_)
    rospy.wait_for_service(go_to_point_switch_)
    rospy.wait_for_service(follow_the_wall_switch_)
    rospy.wait_for_service(random_walk_switch_)
    rospy.wait_for_service(neighbour_status_change_switch_)
    rospy.wait_for_service(environment_switch_)

    srv_leadership_ = rospy.ServiceProxy(neighbour_leadership_switch_, SetBool)
    srv_go_to_point_ = rospy.ServiceProxy(go_to_point_switch_, SetBool2)
    srv_follow_the_wall_ = rospy.ServiceProxy(follow_the_wall_switch_, SetBool2)
    srv_random_walk_ = rospy.ServiceProxy(random_walk_switch_, SetBool)
    srv_status_change_ = rospy.ServiceProxy(neighbour_status_change_switch_, SetBool)
    srv_environment_ = rospy.ServiceProxy(environment_switch_, GroupEnv)

    # initialization of state
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if laser_readings_ == None:
            rate.sleep()
            continue

        # state 0 the leadership estimation
        if state_ == 0:
            leadership_estimation()

        # state 1 go to point
        if state_ == 1:
            if resource_check() == False:
                desired_yaw = math.atan2(shifted_nposition_.y - position_.y, shifted_nposition_.x - position_.x)
                err_yaw = normalize_angle(desired_yaw - yaw_) 
                if ( laser_readings_['front'] < 0.6 or min(laser_readings_['fright'], laser_readings_['fleft']) < 0.45 ) and math.fabs(err_yaw) < math.pi / 2 : #or min(laser_readings_['left'], laser_readings_['right']) < 0.2:
                    if laser_readings_['front'] < 0.6:
                        if err_yaw > 0:
                            follow_side_ = False
                    elif laser_readings_['fright'] < 0.6:
                        follow_side_ = False
                    else:
                        follow_side_ = True
                    change_state(2)

        # state 2 follow the wall
        if state_ == 2:
            if dist_calc() > 1.5 and leader_ == True:
                change_state(3)

            leave_ftw()

        # state 3 waiting for robot (available only for leader)
        if state_ == 3:
            wait()
            wait_time_treshhold_ -=1

        # state 4 random walk
        if state_ == 4:
            if resource_check() == False:
                if dist_calc() > 1.5 and minimal_rw_time_ <= 0:
                    wait_time_treshhold_ = 300
                    change_state(3)

                minimal_rw_time_ -= 1

        # state 5 end of searching / done
        if state_ == 5:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            pub_.publish(msg)

        # state 6 initial leader turn
        if state_ == 6:
            if initial_turn_rand_ == 0:
                initial_turn_rand_ = random.uniform(-1,1)

            msg = Twist()
            msg.linear.x = 0.0 
            msg.angular.z = initial_turn_rand_
            pub_.publish(msg)
            initial_turn_time_ += 1
            if initial_turn_timeout_ == initial_turn_time_:
                change_state(4)
            

        
        rate.sleep()



if __name__ == "__main__":
    main()
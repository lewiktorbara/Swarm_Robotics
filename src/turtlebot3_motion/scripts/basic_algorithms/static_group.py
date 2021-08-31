#! /usr/bin/env python

# That file contains algorithm that makes the robot to don't move

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import *

active_ = False

# publisher
pub_ = None

# robot unit parameters
num_ = rospy.get_param('num')
cmd_vel_ = '/robot_' + str(num_) + '/cmd_vel'
static_switch_ = '/robot_' + str(num_) + '/wait'

# service switch that activates node
def static_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def main():
    global pub_, cmd_vel_, static_switch_

    # node initialization    
    rospy.init_node('static')

    # publishers
    pub_ = rospy.Publisher(cmd_vel_, Twist, queue_size=1)
    
    # services
    srv = rospy.Service(static_switch_, SetBool, static_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0

            pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
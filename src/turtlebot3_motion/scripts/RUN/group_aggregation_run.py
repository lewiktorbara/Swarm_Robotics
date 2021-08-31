#! /usr/bin/env python

# script enables to run serveral simulations in sequence 

import roslaunch
import rospy
import time
from std_srvs.srv import *

run_ = True
sub_ = None

# environment informs that the simulation is finnished 
def switch(req):
    global run_
    run_ = False
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# node initialization
rospy.init_node('sim', anonymous=True)

# services
srv = rospy.Service('/simulation', SetBool, switch)


# forloop that executed n times runs n simulations in a row
for i in range(10):
    run_ = True
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # world launch file path
    simulation = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hairy/ROS/catkin_ws/src/stage_sim/launch/world.launch"])
    # algorithm launch file path
    algorithm = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hairy/ROS/catkin_ws/src/turtlebot3_motion/launch/flocking_rw_10.launch"])
    simulation.start()
    time.sleep(1)
    algorithm.start()
    rospy.loginfo("started")
    
    while run_:
        time.sleep(1)
    time.sleep(1)

    algorithm.shutdown()
    time.sleep(5)
    simulation.shutdown()

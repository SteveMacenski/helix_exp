#!/usr/bin/env python

import rospy
import os
import sys
import std_msgs.msg
from control_msgs.msg import FollowJointTrajectoryActionResult

def callback(results):

    with open('errors.txt','a') as file:
        file.writelines(str(results.status.status) + ' pnt {}\n'.format(results.header.seq))

    

if __name__ == '__main__':
    home = os.path.expanduser('~')
    os.chdir(home+'/catkin_ws/src/helix_exp/experimental_data_and_errors')

    rospy.init_node('error_code_handler', anonymous=True)
    rospy.Subscriber("/rosey/joint_trajectory_action/result",
        FollowJointTrajectoryActionResult, callback)
    rospy.spin()

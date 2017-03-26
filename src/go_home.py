#!/usr/bin/env python

import rospy
import os
import sys
from rospy import Duration
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryActionResult
import time


home_pose = [0,0,0,0,0,0,0,0,0,0,0,0]

#ROS publishers and subscribers
rospy.init_node('hardware_handler', anonymous=True)
pub_gary = rospy.Publisher('/gary/joint_trajectory_action/goal',
     FollowJointTrajectoryActionGoal, queue_size=10) 
pub_rosey = rospy.Publisher('/rosey/joint_trajectory_action/goal',
     FollowJointTrajectoryActionGoal, queue_size=10)
camera_indicator = rospy.Publisher(
    '/measure_shape', std_msgs.msg.String, queue_size=10)
        
#give time to register
time.sleep(4)
               
def go_home():
       # go home
       
       homing_gary = JointTrajectoryPoint()
       homing_rosey = JointTrajectoryPoint()
       homing_rosey.positions = home_pose[0:6]
       homing_gary.positions = home_pose[6:12]
       homing_gary.time_from_start = Duration(0)
       homing_rosey.time_from_start =  Duration(0)
       message_home_gary = FollowJointTrajectoryActionGoal()
       message_home_rosey = FollowJointTrajectoryActionGoal()
       header_home = std_msgs.msg.Header(stamp=rospy.Time.now())
       message_home_gary.goal.trajectory.header = header_home
       message_home_rosey.goal.trajectory.header = header_home
       message_home_gary.header = header_home
       message_home_rosey.header = header_home
       message_home_gary.goal.trajectory.joint_names = ['joint_1', 
                    'joint_2', 'joint_3', 
                    'joint_4', 'joint_5', 'joint_6']
       message_home_rosey.goal.trajectory.joint_names = \
            message_home_gary.goal.trajectory.joint_names
       message_home_gary.goal.trajectory.points = [homing_gary] 
       message_home_rosey.goal.trajectory.points = [homing_rosey] 

       pub_gary.publish(message_home_gary)
       pub_rosey.publish(message_home_rosey)
       
       sys.exit(1)      


if __name__ == '__main__':
    try:
        go_home();
    except rospy.ROSInterruptException:
        pass



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

            #rosey then gary angles
home_pose = [0.,0.25,-0.25,0.,-1.58,2.97, 
             0.,0.25,-0.25,0.,-1.58,2.97]

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

#execute motion of robots
def exection(list_of_trajectories):

    #Hz
    rate = rospy.Rate(0.30) #rate of 0.30 works great for 2s
    #20 min experiment
    
    #rate = rospy.Rate(0.07) #rate of 0.07 works great for 10s
    #1 hr experiment
    
    #count trajectory number
    num_traj = len(list_of_trajectories)
    trajectory_iter = -1
    
    # each n point trajectory
    for trajectory in list_of_trajectories:
    
       if rospy.is_shutdown():
           break
       go_home()
           
       raw_input("Please reset rod and press Enter to continue")
       
       #reset waypoint count and add to trajectory
       trajectory_iter += 1
       waypoint = 0
       
       # each points in trajectory
       for position in trajectory:
            
            #  define and populate a point
            traj_waypoint_gary = JointTrajectoryPoint()
            traj_waypoint_rosey = JointTrajectoryPoint()

            traj_waypoint_gary.positions = position[6:12]    
            traj_waypoint_rosey.positions = position[0:6]

            traj_waypoint_rosey.time_from_start = Duration(2)
            traj_waypoint_gary.time_from_start = Duration(2)
          
            #debug in terminal
            #print 'gary'
            #print traj_waypoint_gary.positions
            #print 'rosey'
            #print traj_waypoint_rosey.positions
         
            #  making message
            message_gary = FollowJointTrajectoryActionGoal()
            message_rosey = FollowJointTrajectoryActionGoal()
           
            #  required headers
            header_gary = \
                std_msgs.msg.Header(stamp=rospy.Time.now())
            header_rosey = \
                std_msgs.msg.Header(stamp=rospy.Time.now())
                    
            message_gary.goal.trajectory.header = header_gary
            message_rosey.goal.trajectory.header = header_rosey
            message_gary.header = header_gary
            message_rosey.header = header_rosey
           
            #  adding in joints
            joint_names = ['joint_1', 'joint_2', 'joint_3', 
                           'joint_4', 'joint_5', 'joint_6']
            message_gary.goal.trajectory.joint_names = joint_names
            message_rosey.goal.trajectory.joint_names = joint_names
           
            #adding points
            if waypoint > 0:
                message_gary.goal.trajectory.points.append( 
                        old_gary_pnt)
                message_rosey.goal.trajectory.points.append(
                        old_rosey_pnt)
                        
            else:
               first_pnt = JointTrajectoryPoint()
               first_pnt.positions = home_pose[0:6]
               first_pnt.time_from_start = Duration(0)
               message_rosey.goal.trajectory.points.append( 
                        first_pnt)
               first_pnt.positions = home_pose[6:12]
               message_gary.goal.trajectory.points.append(
                        first_pnt)            
                 
            message_gary.goal.trajectory.points.append( 
                 traj_waypoint_gary)
            message_rosey.goal.trajectory.points.append(
                 traj_waypoint_rosey)
          
            #  publishing to ROS node
            pub_gary.publish(message_gary)
            pub_rosey.publish(message_rosey)
      
            print waypoint

            if waypoint == 0:
                time.sleep(5)
                rate.sleep()
            rate.sleep()
                
            #take picture and save in format
            title = "T{}P{}".format(trajectory_iter, waypoint)
            camera_indicator.publish(title)
            

            old_gary_pnt = traj_waypoint_gary
            old_gary_pnt.time_from_start = Duration(0)
            old_rosey_pnt = traj_waypoint_rosey
            old_rosey_pnt.time_from_start = Duration(0)
           
            #add to waypoint count
            waypoint += 1
           
            
            if rospy.is_shutdown():
                break
    go_home()
    
               
def go_home():
       # go home
       print home_pose
       
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
       
       #homing_gary.time_from_start = Duration(2)
       #homing_rosey.time_from_start =  Duration(2)
       #homing_rosey.positions = [.5]*6
       #homing_gary.positions = [.5]*6
       #message_home_gary.goal.trajectory.points.append(homing_gary)
       #message_home_rosey.goal.trajectory.points.append(homing_rosey)
       
       pub_gary.publish(message_home_gary)
       pub_rosey.publish(message_home_rosey)
       
       if rospy.is_shutdown():
                sys.exit(1)
       
       
def parse_file(filename):

    f = open(filename,'r')
    trajectories = []
    commands = []
    
    for line in f:
        if line == '\n':
            trajectories.append(commands)
            commands = []
            continue
        individ_positions = [float(element) for element in 
                             line.split(' ') if element != '\n']
        commands.append(individ_positions)
        
    f.close()
    
    if trajectories == []:
        trajectories = [commands]

    return trajectories
            


if __name__ == '__main__':
# READ IN ALL TRAJECTORIES AS LIST OF LISTS OF LISTS FOR EACH RUN
    home = os.path.expanduser('~')
    os.chdir(home+'/catkin_ws/src/helix_exp/experimental_data_and_errors')
    trajectories = parse_file('robot_paths.txt')

# CALL FUNCTION TO LOOP OVER TRAJECTORY, STOP AFTER EACH POINT
        #STARTING FUNCTION CALL CAMERA CALLBACK TO EXECUTE SO ITS 
        #ALWAYS AFTER THE MOVEMENT IS FINISHED.
        # END TRAJ END, SEND HOME
        # WAIT FOR HUMAN INPUT TO REPEAT (2) AND (3)

    try:
        exection(trajectories)
    except rospy.ROSInterruptException:
        pass



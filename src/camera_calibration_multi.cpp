#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include <vector>
#include <fstream>
#include <ctime>
#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> 
#include <string>
#include <stdio.h>
#include <stdlib.h>

//mod this for the path_handler via
  // 1.1 parse file
  // 1.2 calib_images -> real_images

std::vector<double> J1,J2,J3,J4,J5,J6;// vectors to save robo
std::vector<std::vector< double> > cam_poses;// vector to save cam
std::vector<geometry_msgs::PoseArray> cam_orient;// vector to save cam
std::vector<double> Jcurrent(6);
std::vector<std::pair<double,double> > cam_AT_points; // vector to save corner pts
std::vector<std::vector<double> > trajectory; //traj to follow

bool go_tags = false; //moved robot telling tags to work
bool go_robo = false; //saying "im done with moving"
bool tries = true;
bool go_ATraw = false;
bool go_picture = false;
int iterat = 0;

std::vector<double> x_t, y_t, z_t;
int traj_count = 1;
ros::Publisher pub;

void writeToFile(){
    //write file
  std::cout << "writing file" << std::endl;
  std::ofstream myfile;
  
  time_t now = time(0);
  char* dt = ctime(&now);
  
  myfile.open (dt);
  myfile << dt;
  myfile << "\n";
  
  myfile << "camera params: \n";
  myfile << "fx: ";
  myfile << 1412.12646484375;
  myfile << "\nfy: ";
  myfile << 1415.08154296875;
  myfile << "\ncx: ";
  myfile << 1007.9487915039062;
  myfile << "\ncy: ";
  myfile << 553.08056640625;
  
  myfile << "\n\nraw data: \n";
  myfile << "rob pose J1:6\n";
  myfile << "---";
  myfile << "\ncam pose/quat: pose (x,y,z), quaternion qx,qy,qz,qw: \n";
  myfile << "---";
  myfile << "\nray april tags points: p1, p2, p3, p4, cxy: \n\n";
  
  for (int i=0;i!=J1.size();i++){
    myfile << J1[i] << " " << J2[i] << " " << J3[i] << " ";
    myfile << J4[i] << " " << J5[i] << " " << J6[i];
    myfile << "\n";
  }
  myfile << "\n";
  

  for (int i=0;i!=cam_poses.size();i++){
    myfile << cam_poses[i][0] << ", ";
    myfile << cam_poses[i][1] << ", ";
    myfile << cam_poses[i][2] << ", ";
    
    myfile << cam_orient[i].poses[0].orientation.x << ", ";
    myfile << cam_orient[i].poses[0].orientation.y << ", ";
    myfile << cam_orient[i].poses[0].orientation.z << ", ";
    myfile << cam_orient[i].poses[0].orientation.w << ", ";
    myfile << "\n";
  }
  myfile << "\n";

  int myCount = 1;
  for (int i=0;i!=cam_AT_points.size();i++){
    myfile << cam_AT_points[i].first << " " << cam_AT_points[i].second << " "; 
    
    myCount++;
    if (myCount == 6){
      myfile << "\n";
      myCount = 1;
    }
  }
  
  myfile.close();
  ros::shutdown();
}


void printTraj(){

  for (int j=0;j!=10;j++){
    for (int i=0;i!=7;i++){
      std::cout << trajectory[j][i] << " ";
    }
    std::cout << "\n"<< std::endl;
  }
}


void gen_calib_traj(){
  // generates trajectory
   // J1: -25:25 delta:25deg
   // J2: -10:20 delta:5deg
   // J3: -45:45 delta:15deg
   // J4: 90
   // J5: 90
   // J6: 0
   
   trajectory.resize(147);
   for (int i=0;i!=trajectory.size();i++){
     trajectory[i].resize(6);
   }
   
   for(int i=0;i!=trajectory.size();i++){
     if (i<49){
       trajectory[i][0] = -25.;
     }
     if (i<98 && i>=49){
       trajectory[i][0] = 0.;
     }
     if (i<147 && i >=98){
       trajectory[i][0] = 25.;
     }
   }
   
   for(int i=0;i!=49;i++){
     if (i<7){
       trajectory[i][1] = -10.;
       trajectory[i+49][1] = -10.;
       trajectory[i+98][1] = -10.;
     }
     if (i<=14 && i>7){
       trajectory[i][1] = -5.;
       trajectory[i+49][1] = -5.;
       trajectory[i+98][1] = -5.;
     }
     if (i<=21 && i>14){
       trajectory[i][1] = -0.;
       trajectory[i+49][1] = -0.;
       trajectory[i+98][1] = -0.;
     }
     if (i<=28 && i>21){
       trajectory[i][1] = 5.;
       trajectory[i+49][1] = 5.;
       trajectory[i+98][1] = 5.;
     }
     if (i<=35 && i>28){
       trajectory[i][1] = 10.;
       trajectory[i+49][1] = 10.;
       trajectory[i+98][1] = 10.;
     }
     if (i<=42 && i>35){
       trajectory[i][1] = 15.;
       trajectory[i+49][1] = 15.;
       trajectory[i+98][1] = 15.;
     }
     if (i<49 && i>42){
       trajectory[i][1] = 20.;
       trajectory[i+49][1] = 20.;
       trajectory[i+98][1] = 20.;
     }
   }
   
   for(int i=0;i!=7;i++){
     if (i==0){
       trajectory[i][2] = -45.;
       trajectory[i+7][2] = -45.;
       trajectory[i+14][2] = -45.;
       trajectory[i+21][2] = -45.;
       trajectory[i+28][2] = -45.;
       trajectory[i+35][2] = -45.;
       trajectory[i+42][2] = -45.;
       trajectory[i+49][2] = -45.;
       trajectory[i+56][2] = -45.;
       trajectory[i+63][2] = -45.;
       trajectory[i+70][2] = -45.;
       trajectory[i+77][2] = -45.;
       trajectory[i+84][2] = -45.;
       trajectory[i+91][2] = -45.;
       trajectory[i+98][2] = -45.;
       trajectory[i+105][2] = -45.;
       trajectory[i+112][2] = -45.;
       trajectory[i+119][2] = -45.;
       trajectory[i+126][2] = -45.;
       trajectory[i+133][2] = -45.;
       trajectory[i+140][2] = -45.;
     }
     if (i==1){
       trajectory[i][2] = -30.;
       trajectory[i+7][2] = -30.;
       trajectory[i+14][2] = -30.;
       trajectory[i+21][2] = -30.;
       trajectory[i+28][2] = -30.;
       trajectory[i+35][2] = -30.;
       trajectory[i+42][2] = -30.;
       trajectory[i+49][2] = -30.;
       trajectory[i+56][2] = -30.;
       trajectory[i+63][2] = -30.;
       trajectory[i+70][2] = -30.;
       trajectory[i+77][2] = -30.;
       trajectory[i+84][2] = -30.;
       trajectory[i+91][2] = -30.;
       trajectory[i+98][2] = -30.;
       trajectory[i+105][2] = -30.;
       trajectory[i+112][2] = -30.;
       trajectory[i+119][2] = -30.;
       trajectory[i+126][2] = -30.;
       trajectory[i+133][2] = -30.;
       trajectory[i+140][2] = -30.;
     }
     if (i==2){
       trajectory[i][2] = -15.;
       trajectory[i+7][2] = -15.;
       trajectory[i+14][2] = -15.;
       trajectory[i+21][2] = -15.;
       trajectory[i+28][2] = -15.;
       trajectory[i+35][2] = -15.;
       trajectory[i+42][2] = -15.;
       trajectory[i+49][2] = -15.;
       trajectory[i+56][2] = -15.;
       trajectory[i+63][2] = -15.;
       trajectory[i+70][2] = -15.;
       trajectory[i+77][2] = -15.;
       trajectory[i+84][2] = -15.;
       trajectory[i+91][2] = -15.;
       trajectory[i+98][2] = -15.;
       trajectory[i+105][2] = -15.;
       trajectory[i+112][2] = -15.;
       trajectory[i+119][2] = -15.;
       trajectory[i+126][2] = -15.;
       trajectory[i+133][2] = -15.;
       trajectory[i+140][2] = -15.;
     }
     if (i==3){
       trajectory[i][2] = -0.;
       trajectory[i+7][2] = -0.;
       trajectory[i+14][2] = -0.;
       trajectory[i+21][2] = -0.;
       trajectory[i+28][2] = -0.;
       trajectory[i+35][2] = -0.;
       trajectory[i+42][2] = -0.;
       trajectory[i+49][2] = -0.;
       trajectory[i+56][2] = -0.;
       trajectory[i+63][2] = -0.;
       trajectory[i+70][2] = -0.;
       trajectory[i+77][2] = -0.;
       trajectory[i+84][2] = -0.;
       trajectory[i+91][2] = -0.;
       trajectory[i+98][2] = -0.;
       trajectory[i+105][2] = -0.;
       trajectory[i+112][2] = -0.;
       trajectory[i+119][2] = -0.;
       trajectory[i+126][2] = -0.;
       trajectory[i+133][2] = -0.;
       trajectory[i+140][2] = -0.;
     }
     if (i==4){
       trajectory[i][2] = 15.;
       trajectory[i+7][2] =15.;
       trajectory[i+14][2] = 15.;
       trajectory[i+21][2] = 15.;
       trajectory[i+28][2] = 15.;
       trajectory[i+35][2] = 15.;
       trajectory[i+42][2] = 15.;
       trajectory[i+49][2] = 15.;
       trajectory[i+56][2] = 15.;
       trajectory[i+63][2] = 15.;
       trajectory[i+70][2] = 15.;
       trajectory[i+77][2] = 15.;
       trajectory[i+84][2] = 15.;
       trajectory[i+91][2] = 15.;
       trajectory[i+98][2] = 15.;
       trajectory[i+105][2] =15.;
       trajectory[i+112][2] = 15.;
       trajectory[i+119][2] = 15.;
       trajectory[i+126][2] = 15.;
       trajectory[i+133][2] = 15.;
       trajectory[i+140][2] = 15.;
     }
     if (i==5){
       trajectory[i][2] = 30.;
       trajectory[i+7][2] = 30.;
       trajectory[i+14][2] = 30.;
       trajectory[i+21][2] = 30.;
       trajectory[i+28][2] = 30.;
       trajectory[i+35][2] = 30.;
       trajectory[i+42][2] = 30.;
       trajectory[i+49][2] = 30.;
       trajectory[i+56][2] = 30.;
       trajectory[i+63][2] = 30.;
       trajectory[i+70][2] = 30.;
       trajectory[i+77][2] = 30.;
       trajectory[i+84][2] = 30.;
       trajectory[i+91][2] = 30.;
       trajectory[i+98][2] = 30.;
       trajectory[i+105][2] = 30.;
       trajectory[i+112][2] = 30.;
       trajectory[i+119][2] = 30.;
       trajectory[i+126][2] = 30.;
       trajectory[i+133][2] = 30.;
       trajectory[i+140][2] = 30.;
     }
     if (i==6){
       trajectory[i][2] = 45.;
       trajectory[i+7][2] = 45.;
       trajectory[i+14][2] = 45.;
       trajectory[i+21][2] = 45.;
       trajectory[i+28][2] = 45.;
       trajectory[i+35][2] = 45.;
       trajectory[i+42][2] = 45.;
       trajectory[i+49][2] = 45.;
       trajectory[i+56][2] = 45.;
       trajectory[i+63][2] = 45.;
       trajectory[i+70][2] = 45.;
       trajectory[i+77][2] = 45.;
       trajectory[i+84][2] = 45.;
       trajectory[i+91][2] = 45.;
       trajectory[i+98][2] = 45.;
       trajectory[i+105][2] = 45.;
       trajectory[i+112][2] = 45.;
       trajectory[i+119][2] = 45.;
       trajectory[i+126][2] = 45.;
       trajectory[i+133][2] = 45.;
       trajectory[i+140][2] = 45.;
     }
   }

   for(int i=0;i!=trajectory.size();i++){
     trajectory[i][3] = 90.; // -90 for gary
     trajectory[i][4] = 90.;
     trajectory[i][5] = 0.;
   }
   
   for (int i=0;i!=trajectory.size();i++){
     for(int j=0;j!=7;j++){
       trajectory[i][j] += 0; // for artificially creating more pts 
       trajectory[i][j] *= 3.14159/180.;
       
     }
   }
   
  // start at home
  trajectory[0][0] = 0.;
  trajectory[0][1] = 0.;
  trajectory[0][2] = 0.;
  trajectory[0][3] = 1.57; // -1.57 for gary
  trajectory[0][4] = 1.57;
  trajectory[0][5] = 0.;
  
  //printTraj();
}


void moveRobotCaller(){
  // takes in the traj_count and throws next trajectory point to robot
  
  if (traj_count == 101){ //146 total
  
    writeToFile();
  
  } else {

    // condition 
    control_msgs::FollowJointTrajectoryActionGoal msg;
    msg.goal.trajectory.joint_names.push_back("joint_1");
    msg.goal.trajectory.joint_names.push_back("joint_2");
    msg.goal.trajectory.joint_names.push_back("joint_3");
    msg.goal.trajectory.joint_names.push_back("joint_4");
    msg.goal.trajectory.joint_names.push_back("joint_5");
    msg.goal.trajectory.joint_names.push_back("joint_6");
  
    if (traj_count != 1){
      
      if((traj_count == 2) && (tries==true)){
        traj_count--;
        tries = false;
      }
    
      // retrieve pnt
      std::vector<double> old_waypnt(trajectory[traj_count-1]);
      std::vector<double> waypnt(trajectory[traj_count]);
  
      msg.goal.trajectory.points.resize(2);
  
      msg.goal.trajectory.points[0].positions.resize(6);
      msg.goal.trajectory.points[0].positions[0] = old_waypnt[0];
      msg.goal.trajectory.points[0].positions[1] = old_waypnt[1];
      msg.goal.trajectory.points[0].positions[2] = old_waypnt[2];
      msg.goal.trajectory.points[0].positions[3] = old_waypnt[3];
      msg.goal.trajectory.points[0].positions[4] = old_waypnt[4];
      msg.goal.trajectory.points[0].positions[5] = old_waypnt[5];
    
      msg.goal.trajectory.points[1].positions.resize(6);
      msg.goal.trajectory.points[1].positions[0] = waypnt[0];
      msg.goal.trajectory.points[1].positions[1] = waypnt[1];
      msg.goal.trajectory.points[1].positions[2] = waypnt[2];
      msg.goal.trajectory.points[1].positions[3] = waypnt[3];
      msg.goal.trajectory.points[1].positions[4] = waypnt[4];
      msg.goal.trajectory.points[1].positions[5] = waypnt[5];
   
      msg.goal.trajectory.points[0].time_from_start = \
                                     ros::Duration(0.);
      msg.goal.trajectory.points[1].time_from_start = \
                                     ros::Duration(1.9);
                                     
      // send
      std::cout << "sending! " << traj_count << std::endl;
      pub.publish(msg);
    
      // index up
      traj_count++;
      
    } else {
    
      // retrieve pnt
      std::vector<double> old_waypnt(trajectory[traj_count-1]);
      
      // first move go to AprilTag home in 10 s
      msg.goal.trajectory.points.resize(1);
  
      msg.goal.trajectory.points[0].positions.resize(6);
      msg.goal.trajectory.points[0].positions[0] = old_waypnt[0];
      msg.goal.trajectory.points[0].positions[1] = old_waypnt[1];
      msg.goal.trajectory.points[0].positions[2] = old_waypnt[2];
      msg.goal.trajectory.points[0].positions[3] = old_waypnt[3];
      msg.goal.trajectory.points[0].positions[4] = old_waypnt[4];
      msg.goal.trajectory.points[0].positions[5] = old_waypnt[5];
      
      msg.goal.trajectory.points[0].time_from_start = \
                                     ros::Duration(10.);
    
      // send first msg to register
      std::cout << "sending!" << std::endl;
      pub.publish(msg);
 
      // pause for registration
      unsigned int seconds2 = 2;
      sleep(seconds2);
      
      // go move second call
      pub.publish(msg);

      traj_count++;
    }

    go_tags = true;
  }
}


void poseCallback(const geometry_msgs::PoseArray& pose){

  double x_c=0,y_c=0,z_c=0;
  double x_cam, y_cam, z_cam;

  // if robot says to take data
  if ((go_tags==true) && (pose.poses.size() > 0) && (go_robo==true)){
  
    x_t.push_back(pose.poses[0].position.x);
    y_t.push_back(pose.poses[0].position.y);
    z_t.push_back(pose.poses[0].position.z);
    std::cout << "got: " << x_t.size() << std::endl;
    if (x_t.size() >= 3){

      for(int i=1;i<x_t.size();i++){
        x_c += x_t[i];
        y_c += y_t[i];
        z_c += z_t[i];
      }

      x_cam = x_c/(x_t.size()-1);
      y_cam = y_c/(y_t.size()-1);
      z_cam = z_c/(z_t.size()-1);
      std::vector<double> cam_pose(3);
      cam_pose[0] = x_cam;
      cam_pose[1] = y_cam;
      cam_pose[2] = z_cam;
      cam_poses.push_back(cam_pose);
      
      cam_orient.push_back(pose);
      
      //trigger next robot position move & save pose
      J1.push_back(Jcurrent[0]);
      J2.push_back(Jcurrent[1]);
      J3.push_back(Jcurrent[2]);
      J4.push_back(Jcurrent[3]);
      J5.push_back(Jcurrent[4]);
      J6.push_back(Jcurrent[5]);
      
      go_tags = false;
      go_robo = false;
      go_ATraw = true;
      x_t.clear(); y_t.clear(); z_t.clear();
      moveRobotCaller();
    } else if (x_t.size() == 2){
      go_ATraw = true;
      go_picture = true;
    }  
    
  } 
}


void jointCallback(const sensor_msgs::JointState& Jpose){
  // get current joint angles in global var
  Jcurrent[0] = Jpose.position[0];
  Jcurrent[1] = Jpose.position[1];
  Jcurrent[2] = Jpose.position[2];
  Jcurrent[3] = Jpose.position[3];
  Jcurrent[4] = Jpose.position[4];
  Jcurrent[5] = Jpose.position[5];
}


void statusCallback(const \
      control_msgs::FollowJointTrajectoryActionResult& result){
      // on completion of move, turn true
  std::cout << "done moving" << std::endl;
      
  if (result.status.status == 3){
    go_robo = true;
  }
}


void ATCallback(const geometry_msgs::PoseArray& ATpoints){
  // take data on the detected April tags raw, not the pose estimation
  // info is ordered: p1, p2, p3, p4, cxy
  
  if (go_ATraw == true){
    for (int i=0;i!=5;i++){
      cam_AT_points.push_back( \
                    std::make_pair( \
                    ATpoints.poses[i].position.x, \
                    ATpoints.poses[i].position.y)); 
    }
    go_ATraw = false;
  }
}


void imageCallback(const sensor_msgs::Image& image){

  if (go_picture == true){

    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(image, "bgr8");

    std::stringstream sstream;
  
   sstream << "./../../../../catkin_ws/src/helix_exp/experimental_data_and_errors/calib_images/" << iterat << ".bmp" ;
   ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );
  
    go_picture = false;
    iterat += 1;
  }
}


int main(int argc, char **argv){  

  // initialize ROS
  ros::init(argc, argv, "caller"); 
  ros::NodeHandle m;   
  ros::Subscriber sub = \
     m.subscribe("/tag_detections_pose", 100, poseCallback);
  ros::Subscriber subjoint = \
     m.subscribe("/rosey/joint_states", 100, jointCallback); //gary
  ros::Subscriber substatus = \
     m.subscribe("/rosey/joint_trajectory_action/result", 100, \
      statusCallback); //gary
  ros::Subscriber subAT = \
     m.subscribe("AprilTagPointsRaw", 100, \
      ATCallback);
  ros::Subscriber subimage = m.subscribe("/usb_cam/image_raw", 50, &imageCallback);
  pub = m.advertise \
     <control_msgs::FollowJointTrajectoryActionGoal> \
     ("/rosey/joint_trajectory_action/goal", 10); //gary
        
  // generate trajectory
  gen_calib_traj();

  moveRobotCaller();

  // book keeping
  ros::spin();
  return 0;
}

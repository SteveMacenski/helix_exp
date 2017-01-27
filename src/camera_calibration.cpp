#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include <vector>
#include <fstream>
#include <ctime>

std::vector<double> x_t;
std::vector<double> y_t;
std::vector<double> z_t;

double J1,J2,J3,J4,J5,J6;

std::vector<std::vector<double> > quatPose2Tform(
                 const geometry_msgs::Quaternion q,
                 const std::vector<double> translation){

  double xx = q.x*q.x, xy = q.x*q.y, xz = q.x*q.z, xw = q.x*q.w;
  double yy = q.y*q.y, yz = q.y*q.z, yw = q.y*q.w;
  double zz = q.z*q.z, zw = q.z*q.w;

  std::vector< std::vector< double> > rotation(4, std::vector<double>(4));

  rotation[0][0] = (1 - 2 * yy - 2 * zz);
  rotation[0][1] = (2 * xy - 2 * zw);
  rotation[0][2] =  (2 * xz + 2 * yw);
  rotation[1][0] = (2 * xy + 2 * zw);
  rotation[1][1] = (1 - 2 * xx - 2 * zz);
  rotation[1][2] = (2 * yz - 2 * xw);
  rotation[2][0] = (2 * xz - 2 * yw);
  rotation[2][1] = (2 * yz + 2 * xw);
  rotation[2][2] = (1 - 2 * xx - 2 * yy);
  rotation[0][3] = translation[0];
  rotation[1][3] = translation[1];
  rotation[2][3] = translation[2];
  rotation[3][3] = 1;
  rotation[3][0] = 0;
  rotation[3][1] = 0;
  rotation[3][2] = 0;
  
  return rotation;
}

std::vector<std::vector<double > > invertT(std::vector<std::vector<double> > Transform){

 //extract rot matrix
  std::vector<std::vector<double> > rotation_output(3, 
                                    std::vector<double>(3));
  rotation_output[0][0] = Transform[0][0];
  rotation_output[0][1] = Transform[1][0];
  rotation_output[0][2] = Transform[2][0];
  rotation_output[1][0] = Transform[0][1];
  rotation_output[1][1] = Transform[1][1];
  rotation_output[1][2] = Transform[2][1];
  rotation_output[2][0] = Transform[0][2];
  rotation_output[2][1] = Transform[1][2];
  rotation_output[2][2] = Transform[1][1];
    
  std::vector<double> T_output(3);
  T_output[0]=-1*(rotation_output[0][0]*Transform[0][3] + \
    rotation_output[0][1]*Transform[1][3] + \
    rotation_output[0][2]*Transform[2][3]);
    
  T_output[1]=-1*(rotation_output[1][0]*Transform[0][3] + \
    rotation_output[1][1]*Transform[1][3] + \
    rotation_output[1][2]*Transform[2][3]);
    
  T_output[2]=-1*(rotation_output[2][0]*Transform[0][3] + \
    rotation_output[2][1]*Transform[1][3] + \
    rotation_output[2][2]*Transform[2][3]);
  
 
  
  std::vector<std::vector<double> > output(4, std::vector<double>(4));
  output[0][0] = rotation_output[0][0];
  output[1][0] = rotation_output[1][0];
  output[2][0] = rotation_output[2][0];
  output[0][1] = rotation_output[0][1];
  output[1][1] = rotation_output[1][1];
  output[2][1] = rotation_output[2][1];
  output[0][2] = rotation_output[0][2];
  output[1][2] = rotation_output[1][2];
  output[2][2] = rotation_output[2][2];
  
  output[0][3] = T_output[0];
  output[1][3] = T_output[1];
  output[2][3] = T_output[2];
  
  output[3][3] = 1;
  output[3][0] = 0;
  output[3][1] = 0;  
  output[3][2] = 0;

  
  return output;
}

std::vector<std::vector<double> > multiT(
            std::vector<std::vector<double> > T1,
            std::vector<std::vector<double> > T2){
            
  std::vector<std::vector<double> > output(4, std::vector<double>(4));

  for (int j=0;j<4;++j){
    for (int k=0;k<4;++k){
      for (int i=0;i<4;++i){
        output[i][j] += T1[i][k]*T2[k][j];
      }
    }
  }
    
  return output;
}


void print_matrix(std::vector<std::vector<double> > output){

    for (int i = 0; i < output.size(); i++){
          std::cout << " " << std::endl;
      for (int j = 0; j < output[i].size(); j++){
        std::cout << output[i][j];
        std::cout << " ";
      }
    }
}


void interaction(){
  //wait for user to input the position of robot
  std::cout << "Please enter J1 value in m from pendant for Rosey: ";
  std::cin >> J1;
  std::cout << "Please enter J2 value in m from pendant for Rosey: ";
  std::cin >> J2;
  std::cout << "Please enter J3 value in m from pendant for Rosey: ";
  std::cin >> J3;
  std::cout << "Please enter J4 value from pendant for Rosey: ";
  std::cin >> J4;
  std::cout << "Please enter J5 value from pendant for Rosey: ";
  std::cin >> J5;
  std::cout << "Please enter J6 value from pendant for Rosey: ";
  std::cin >> J6;

}

void poseCallback(const geometry_msgs::PoseArray& pose){

  x_t.push_back(pose.poses[0].position.x);
  y_t.push_back(pose.poses[0].position.y);
  z_t.push_back(pose.poses[0].position.z);
  
  if (x_t.size() > 5){
    double x_c,y_c,z_c;
    double x_cam, y_cam, z_cam;
    
    for(int i=0;i<x_t.size();i++){
      x_c += x_t[i];
      y_c += y_t[i];
      z_c += z_t[i];
    }
    
    x_cam = x_c/x_t.size();
    y_cam = y_c/y_t.size();
    z_cam = z_c/z_t.size();
    std::vector<double> cam_pose(3);
    cam_pose[0] = x_cam;
    cam_pose[1] = y_cam;
    cam_pose[2] = z_cam;
    
    std::vector< std::vector< double> > cam2tagT(4, std::vector<double>(4));
    cam2tagT = quatPose2Tform(pose.poses[0].orientation, cam_pose);
    
    //TODO doesnt account for EE
    /*
    std::vector<double> rob_pose(3);
    rob_pose[0] = x; rob_pose[1] = y; rob_pose[2] = z;

    geometry_msgs::Quaternion rob_orientation;
    rob_orientation.x=qx; rob_orientation.y=qy; 
    rob_orientation.z=qz; rob_orientation.w=qw;
    
    std::vector< std::vector< double> > rob2tagT(4, std::vector<double>(4));
    rob2tagT = quatPose2Tform(rob_orientation, rob_pose);
    
    std::vector< std::vector< double> > tag2robT(4, std::vector<double>(4));
    tag2robT = invertT(rob2tagT);
    
   std::vector< std::vector< double> > cam2robT(4, std::vector<double>(4));
    cam2robT = multiT(cam2tagT, tag2robT); //This is the thing we want
    print_matrix(cam2robT);
    
    */
    
    //write file
    std::cout << "writing file" << std::endl;
  std::ofstream myfile;
  
  time_t now = time(0);
  char* dt = ctime(&now);
  
  myfile.open (dt);
  myfile << dt;
  myfile << "\n\n";
  
  
  myfile << "camera params: \n";
  myfile << "fx: \n";
  myfile << 1412.12646484375;
  myfile << "\nfy: \n";
  myfile << 1415.08154296875;
  myfile << "\ncx: \n";
  myfile << 1007.9487915039062;
  myfile << "\ncy: \n";
  myfile << 553.08056640625;
  
  myfile << "\n\nraw data: \n";
  myfile << "rob pose \n";
  myfile << "J1: " << J1;
  myfile << "\n";
    myfile << "J2: " << J2;
  myfile << "\n";
    myfile << "J3: " << J3;
  myfile << "\n";
    myfile << "J4: " << J4;
  myfile << "\n";
    myfile << "J5: " << J5;
  myfile << "\n";
    myfile << "J6: " << J6;
  myfile << "\n";

  myfile << "\ncam pose/quat: \n";
  myfile << "x pose:";
  myfile << x_cam;
  myfile << "\n";
  myfile << "y pose:";
  myfile << y_cam;
  myfile << "\n";
  myfile << "z pose:";
  myfile << z_cam;
  myfile << "\n";
  myfile << "quaternion qx,qy,qz,qw:\n";
  myfile << pose.poses[0].orientation.x;
  myfile << "\n";
  myfile << pose.poses[0].orientation.y;
  myfile << "\n";
  myfile << pose.poses[0].orientation.z;
  myfile << "\n";
  myfile << pose.poses[0].orientation.w;
  myfile << "\n";
  
  myfile.close();

       
    ros::shutdown();
  }
}


int main(int argc, char **argv)
{  
  ros::init(argc, argv, "camera_calibration"); 
  ros::NodeHandle m;
  
  interaction();

  ros::Subscriber sub = m.subscribe("/tag_detections_pose", 100, poseCallback);
    
  ros::spin();
  return 0;
}

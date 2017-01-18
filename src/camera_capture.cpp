#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <unistd.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> 
#include <unistd.h>

//globals
cv_bridge::CvImagePtr cv_ptr; 
static const std::string OPENCV_WINDOW = "video feed";

//camera feed request callback function
void cameraCallback(const std_msgs::String& image_title)
{

  std::stringstream sstream;
  
  sstream << "./../../../../catkin_ws/src/helix_exp/experimental_data_and_errors/result_images/" << image_title << ".bmp" ;
  ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );
  
  //call CV and shape estimation in real time?

}

//camera feed update callback function
void imageCallback(const sensor_msgs::Image& image)
{
  cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
  
  //view livestream
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);
}


int main(int argc, char **argv)
{  
  ros::init(argc, argv, "camera_capture"); 
  ros::NodeHandle m;

  ros::Subscriber sub = m.subscribe("/measure_shape", 1000, cameraCallback);
  
  ros::Subscriber sub2 = m.subscribe("/camera/color/image_raw", 50, &imageCallback);
  
  cv::namedWindow(OPENCV_WINDOW);
  
  ros::spin();
  return 0;
}



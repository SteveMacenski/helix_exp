#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>

using namespace cv;


//TODO rotate image 90 degrees, then myROI for new setup w. horiz. cam. data in ID/T

Mat Threshold( Mat& src, char* window_name )
{
  int threshold_value = 54;
  int const max_BINARY_value = 255;
  int threshold_type = 1;

  threshold( src, src, threshold_value, max_BINARY_value,threshold_type );
  
  Rect myROI(250, 50, 1050, 1000);
  src = src(myROI);
  
  imshow( window_name, src );
  
  return src;
}

Mat Find_Rod( Mat& src, char* window_name2, char* file_name )
{
  std::vector<int> equipment_forward;
  std::vector<int> equipment_backward;
  equipment_forward.resize(src.cols);
  equipment_backward.resize(src.cols);
  std::vector<int> rod;
  double percentCoverage = .02;
  int is_equip;
  int smoothing = 70;
  int starting, ending;
  Mat source;
  source = src.clone();
  bool first = false;
  int hor_starting, hor_ending;
  int hor_height, height;
  
  //for all pixels sweeping vertically forwards
  for (int col = 0; col < src.cols; col++){
    equipment_forward[col] = 0;
    for (int row = 0; row < src.rows; row++){
      if (src.at<uchar>(col,row) == 0){
         equipment_forward[col]++; 
      }
    }

    if (equipment_forward[col] > percentCoverage*src.rows){
      // this is potentially robot
      
      // smoothing bands in middle out, but more EE2 showing
      if (col > smoothing){
        is_equip = 0;
        for (int past=0; past < smoothing; past++){
          if (equipment_forward[col-past] > percentCoverage*src.rows){
            is_equip++;
          }
        } 
        if (is_equip <= smoothing*0.5){
          //actually rod
          continue;
        }  
      }
      
      //to visualize what it marked, last after smoothing
      for (int row = 0; row <= src.rows; row++){
        src.at<uchar>(col, row) = 0;
      }
    }
  }
  
  
  //for all pixels sweeping vertically backwards
  for (int col = src.cols; col > 0; col--){
    equipment_backward[src.cols-col] = 0;
    for (int row = 0; row < src.rows; row++){
      if (src.at<uchar>(col,row) == 0){
         equipment_backward[src.cols-col]++; 
      }
    }

    if (equipment_backward[src.cols-col] > percentCoverage*src.rows){
      // this is potentially robot

      // smoothing bands in middle out, kills EE2
      if (col < src.cols-smoothing){
        is_equip = 0;
        
        for (int past=smoothing; past > 0; past--){

          if (equipment_backward[src.cols-col-past] > \
                         percentCoverage*src.rows){
            is_equip++;
          }
        } 
        
        if (is_equip <= smoothing*0.5){
          //actually rod
          continue;
        }  
      }

      //to visualize what it marked, last after smoothing
      for (int row = 0; row <= src.rows; row++){
        src.at<uchar>(col, row) = 0;
      }
    }
  } 

  //imwrite("horizontal_band_im.bmp", src);


  // remove black pixels so that its just remaining ones
  
  for (int col = 0; col < src.cols; col++){
    if (equipment_forward[col] < percentCoverage*src.rows && col > 50){
      starting = col;
      break;
      }
    }
    
  for (int col = 0; col < src.cols; col++){
    if (equipment_backward[col] < src.rows && col > 50){
      ending = src.cols - col;
      break;
      }
    }
    
  height = ending-starting-smoothing-2; //2 to get rid of pot. bars
  Rect myROI(0, starting+2, src.cols, height);
  src = src(myROI);
  
  //imwrite("crop_horizontal_im.bmp", src);
  
  // for remaining pixels, sweep horizontally
  rod.resize(src.cols);
  
  for (int row = 0; row < src.cols; row++){
    rod[row] = 0; 
    for (int col = 0; col < src.rows; col++){
      if (src.at<uchar>(col,row) == 0){
         rod[row]++; 
      }
    }

    if (rod[row] < percentCoverage*src.rows/4){
      //potentially rod
      for (int col = 0; col < src.cols; col++){
        src.at<uchar>(col, row) = 0;
      }
    }

  }

  //imwrite("vertical_band_im.bmp", src);
  
  // remove black pixels so that its just remaining ones
  for (int row = 0; row < src.cols; row++){
    if (rod[row] > percentCoverage*src.rows/4 && row > 5){
      if (first == false){
        hor_starting = row;
        first = true;
      }
      hor_ending = row;
      }
    }
    
  hor_height = hor_ending-hor_starting-7; //tuned, 10 works too
  Rect ROI(hor_starting, 0, hor_height, src.rows);
  src = src(ROI);

  //imwrite("vertical_crop_im.bmp", src);
  
  // subsample points on the curve evenly
  
  std::vector<cv::Point> rod_points;
  std::vector<cv::Point> rod_points_subsampled_rodF;
  std::vector<cv::Point> rod_points_subsampled_picF;
  
  int sub_ROI = 50;
  int x = src.rows/sub_ROI;
  Mat currentROI;
  
  for (int row=1; row < src.rows/x; row++){
    
    //sets ROI and slices to it
    Rect ROIsize(0, row*x, src.cols, 4);
    currentROI = src(ROIsize);
      
    //clears the point count
    rod_points.clear();
      
    //finds points in ROI
    for (int colI = 0; colI < currentROI.cols; colI++){
      for (int rowI = 0; rowI < currentROI.rows; rowI++){
        if (currentROI.at<uchar>(rowI,colI) == 0){
          rod_points.push_back(Point(colI,rowI)); 
        }
      }
    }
    if (rod_points.size()>10){
    //with points find center pixel
      int x_pos = 0;
      int y_pos = 0;
      for(int j=0;j<rod_points.size();j++){
        x_pos += rod_points[j].x;
        y_pos += rod_points[j].y;
      }
      x_pos /= rod_points.size();
      y_pos /= rod_points.size();
        
      rod_points_subsampled_rodF.push_back(Point(x_pos, y_pos+row*x));
      rod_points_subsampled_picF.push_back(Point(x_pos+hor_starting,
                                             y_pos+row*x+starting));
        
      circle(src,Point(x_pos, y_pos+row*x),x,1);
      circle(source,Point(x_pos +hor_starting,
                                         y_pos+row*x+starting),x,1);
    }
  }
  

  // show the ROI on the original image
  rectangle(source,Point(hor_starting,starting), 
                            Point(hor_ending-7,ending-72), 1);
  namedWindow( "ROI", 1000 );
  imshow( "ROI", source );
  
  //save pnts+ROI over source threshold for troubleshooting
  std::string file_write = std::string(file_name);
  size_t lastIndex = file_write.find_last_of(".");
  std::string fileName = file_write.substr(0,lastIndex);
  std::string Tpoint = fileName;
  fileName.append("ROIandPnts");
  fileName.append(".jpg");
  imwrite(fileName, source, (std::vector<int>){CV_IMWRITE_JPEG_QUALITY,0});
  
  //write to file 1 pnt/line, 1 space between rod shapes, w/ date
  
  time_t now = time(0);
  char* dt = ctime(&now);
  
  std::fstream myfile;
  myfile.open("rodPoints.txt", std::ios::in | std::ios::out | std::ios::ate);
  std::cout << Tpoint << std::endl;
  myfile << Tpoint;
  myfile << " Date: ";
  myfile << dt;
  for(int h=0;h<rod_points_subsampled_picF.size();h++){
    myfile << rod_points_subsampled_picF[h];
    myfile << "\n";
  }
  
  myfile.close();

  // show the rod with ROI and points 
  imshow( window_name2, src );
  return src;
}

/*Mat flip_90(Mat& src){

  for(int i = 0; i != 2; ++i){
    transpose(src, src);   
    flip(src, src, 0);
  }
  
  Rect myROI(350, 20, 1250, 880);
  rectangle(src,Point(350,20),Point(1600,900),200);
  src = src(myROI);
  
  imshow( window_name, src );
  
  return src;

}
*/

int main( int argc, char** argv )
{
  Mat src; Mat src_gray; Mat src_thresh_gray;
  Mat seg_rod;
 
  src = imread(argv[1], 1 );
  
  ///flip sideways to work
  //src=flip_90(src);

  /// Convert the image to Gray
  cvtColor( src, src_gray, CV_BGR2GRAY );
  //imwrite("gray_scaled_im.bmp", src_gray);
  
  /// Create a window to display results
  char* window_name = "Threshold Robot";
  char* window_name2 = "Segment Out Robot";
  namedWindow( window_name, 1000 );
  namedWindow( window_name2, 1000 );
  
  /// threshold the grey scaled image
  src_thresh_gray = Threshold( src_gray, window_name );
  //imwrite("thresh_im.bmp", src_thresh_gray);
  
  /// Loop over each vertical, % above N, is robot
  seg_rod = Find_Rod( src_thresh_gray, window_name2, argv[1] );
  
  waitKey(0);
  return(0);
  /// Wait until user finishes program
  while(true)
  {
    int c;
    c = waitKey( 20 );
    if( (char)c == 27 )
      { break; }
   }
}

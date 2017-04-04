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
#include <algorithm>
#include <unistd.h>

using namespace cv;

Mat flip_90(Mat& src){


  transpose(src, src);   
  flip(src, src, 0);

  Rect myROI(0, 500, 1080, 1100);
  src = src(myROI);
  
  return src;

}

Mat flip_270(Mat& src){


  transpose(src, src);   
  flip(src, src, 0);
  
  return src;

}

Mat Threshold( Mat& src, char* window_name )
{
  int threshold_value = 70;
  int const max_BINARY_value = 255;
  int threshold_type = 1;

  threshold( src, src, threshold_value, max_BINARY_value,threshold_type );
  
  Rect myROI(0, 0, 1080-80, 1000);
  src = src(myROI);
  
                //imshow( window_name, src );
  
  return src;
}

bool point_comparator(const cv::Point &a, const cv::Point &b) {
    return (a.x < b.x);
}

void Find_Rod( Mat& src, char* window_name2, char* file_name )
{

  std::vector<int> equipment_forward;
  std::vector<int> equipment_backward;
  equipment_forward.resize(src.cols);
  equipment_backward.resize(src.cols);
  std::vector<int> rod;
  double percentCoverage = .012; // was 0.02
  int is_equip;
  int smoothing = 20;
  int starting, ending;
  Mat source;
  Mat source_rod = src.clone();
  source = src.clone();
  bool first = false;
  int hor_starting, hor_ending;
  int hor_height, height;
  Mat hor_src;
  Mat vert_src;
  Mat currentROI;

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
        if (is_equip <= smoothing*0.9){
          //actually rod
          continue;
        }  
      }
      
      //to visualize what it marked, last after smoothing
      for (int row = 0; row <= src.rows; row++){
        //src.at<uchar>(col, row) = 1;
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
        
        if (is_equip <= smoothing*0.9){
          //actually rod
          continue;
        } 
      }

      //to visualize what it marked, last after smoothing
      for (int row = 0; row <= src.rows; row++){
        //src.at<uchar>(col, row) = 1;
      }
    }
  } 

  // remove black pixels so that its just remaining ones

  
  for (int col = 0; col < src.cols; col++){
    if (equipment_forward[col] < percentCoverage*src.rows && col > 50){
      starting = col;
          //TODO make starting/ending false positive proof
      break;
    }
  }

  for (int col = 0; col < src.cols; col++){
    if ((equipment_backward[col] < percentCoverage*src.rows) && (col > 50)){
      ending = src.cols - col;
      break;
    }
  }

  height = ending-starting-smoothing-2; //2 to get rid of pot. bars
  Rect myROI(0, starting+2, src.cols, height);

  hor_src = src(myROI);

                //namedWindow( "horizontal_band_im.bmp", 1000 );
                //imshow("horizontal_band_im.bmp", hor_src); 
                //imwrite("crop_horizontal_im.bmp", hor_src);

  // for remaining pixels, sweep horizontally
  rod.resize(hor_src.cols);
  
  for (int row = 0; row < hor_src.cols; row++){
    rod[row] = 0; 
    for (int col = 0; col < hor_src.rows; col++){
      if (hor_src.at<uchar>(col,row) == 0){
         rod[row]++; 
      }
    }

    if (rod[row] < percentCoverage*hor_src.rows/4){
      //potentially rod
      for (int col = 0; col < hor_src.cols; col++){
        //hor_src.at<uchar>(col, row) = 0; // TODO why removed
      }
    }

  }

  // remove black pixels so that its just remaining ones
  for (int row = 0; row < hor_src.cols; row++){
    if (rod[row] > percentCoverage*hor_src.rows/4 && row > 50){
      if (first == false){
        hor_starting = row;
        first = true;
      }
      hor_ending = row;
      }
    }
    
  hor_height = hor_ending-hor_starting-0; //7-10 works
  Rect ROI(hor_starting, 0, hor_height, hor_src.rows);

  vert_src = hor_src(ROI);

                //namedWindow( "vertical_band_im.bmp", 1000 );
                //imshow("vertical_band_im.bmp", vert_src); 
                //imwrite("Avertical_crop_im.bmp", vert_src);
  
  //////////////////////////////////////////////////////////////
  
  // subsample points on the curve evenly
  
  std::vector<cv::Point> rod_points;
  std::vector<cv::Point> rod_points_subsampled_rodF;
  std::vector<cv::Point> rod_points_subsampled_picF;
  
  int sub_ROI = 50;
  int x = vert_src.rows/sub_ROI;

  for (int row=1; row < vert_src.rows/x-1; row++){
    //sets ROI and slices to it
    Rect ROIsize(0, row*x, vert_src.cols, 4);
    currentROI = vert_src(ROIsize);

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


    ////////////////////////////////////////////////////////////////
    
    
    if (rod_points.size()>10){
    //with points find center pixel, 10 for removing small noise bits
    
      // sorted list of points
      // std::vector<cv::Point> sorted_rod_points(rod_points);
      // std::sort(sorted_rod_points.begin(), sorted_rod_points.end(), 
                                                 //point_comparator);
           
      // if distance between highest and lowest is high      
      /* if (abs(sorted_rod_points.front().x-
                   sorted_rod_points.back().x) > 30){
        
        // if there may be a 3rd cluster
        if (abs(sorted_rod_points.front().x-sorted_rod_points[sorted_rod_points.size()/2].x) > 30 && 
            abs(sorted_rod_points.back().x-sorted_rod_points[sorted_rod_points.size()/2].x) > 30){
          std::cout << "THREE CLUSTERS" << std::endl;
          //3 clusters
          int max_distance = 0;
          int secondary_distance = 0;
          int split_index;
          int secondary_index;
          for (int g=0;g<sorted_rod_points.size()-1; g++){
          //check max distance
            if (abs(sorted_rod_points[g].x-sorted_rod_points[g+1].x) > max_distance){
              max_distance = abs(sorted_rod_points[g].x-sorted_rod_points[g+1].x);
              split_index = g;
            }
            else{
              if (abs(sorted_rod_points[g].x-sorted_rod_points[g+1].x) > secondary_distance){
                secondary_distance =abs(sorted_rod_points[g].x-sorted_rod_points[g+1].x);
                secondary_index=g;
              }
            }
          }
          //new vector for each cluster
          if (secondary_index < split_index){
            int temp = split_index;
            split_index = secondary_index;
            secondary_index = split_index;
          }

          std::vector<cv::Point> c1(sorted_rod_points);
          std::vector<cv::Point> c2(sorted_rod_points);
          std::vector<cv::Point> c3(sorted_rod_points);
          //end
          c1.erase(c1.begin(), c1.begin()+split_index+secondary_index);
          //beginning
          c2.erase(c2.begin()+split_index, c2.end());
          //middle
          c3.erase(c3.begin(), c3.begin()+split_index);
          c3.erase(c3.begin()+secondary_index-split_index, c3.end());

          int x_pos = 0;
          int y_pos = 0;
          for (int k=0;k<c1.size();k++){
            x_pos += c1[k].x;
            y_pos += c1[k].y;
          }
          x_pos /= c1.size();
          y_pos /= c1.size();

          rod_points_subsampled_rodF.push_back(Point(x_pos,y_pos+row*x));
          rod_points_subsampled_picF.push_back(Point(x_pos+hor_starting, 
                                                 y_pos+row*x+starting));
          circle(src,Point(x_pos, y_pos+row*x),x,1);
          circle(source,Point(x_pos +hor_starting, 
                                  y_pos+row*x+starting),x,1);

          x_pos=0;y_pos=0;
          for (int k=0;k<c2.size();k++){
            x_pos += c2[k].x;
            y_pos += c2[k].y;
          }
          x_pos /= c2.size();
          y_pos /= c2.size();

          rod_points_subsampled_rodF.push_back(Point(x_pos, y_pos+row*x));
          rod_points_subsampled_picF.push_back(Point(x_pos+hor_starting,
                                                 y_pos+row*x+starting));

          circle(src,Point(x_pos, y_pos+row*x),x,1);
          circle(source,Point(x_pos+hor_starting, 
                                  y_pos+row*x+starting),x,1);

          x_pos=0;y_pos=0;
          for (int k=0;k<c3.size();k++){
            x_pos += c3[k].x;
            y_pos += c3[k].y;
          }
          x_pos /= c3.size();
          y_pos /= c3.size();

          rod_points_subsampled_rodF.push_back(Point(x_pos, y_pos+row*x));
          rod_points_subsampled_picF.push_back(Point(x_pos+hor_starting,
                                                 y_pos+row*x+starting));

          circle(src,Point(x_pos, y_pos+row*x),x,1);
          circle(source,Point(x_pos+hor_starting, 
                        y_pos+row*x+starting),x,1);              
          }
      
      
      
        else{ 
          std::cout << "TWO CLUSTERS" << std::endl;
          // 2 clusters
          
          int max_distance = 0;
          int split_index;
          for (int g=0;g<sorted_rod_points.size()-1; g++){
          //check max distance
            if (abs(sorted_rod_points[g].x-sorted_rod_points[g+1].x) > max_distance){
              max_distance = abs(sorted_rod_points[g].x-sorted_rod_points[g+1].x);
              split_index = g;
            }
          }
          //new vector for each cluster
          std::vector<cv::Point> c1(sorted_rod_points);
          std::vector<cv::Point> c2(sorted_rod_points);
          c1.erase(c1.begin(), c1.begin()+split_index);
          c2.erase(c2.begin()+split_index, c2.end());

          int x_pos = 0;
          int y_pos = 0;
          for (int k=0;k<c1.size();k++){
            x_pos += c1[k].x;
            y_pos += c1[k].y;
          }
          x_pos /= c1.size();
          y_pos /= c1.size();

          rod_points_subsampled_rodF.push_back(Point(x_pos,y_pos+row*x));
          rod_points_subsampled_picF.push_back(Point(x_pos+hor_starting, 
                                                 y_pos+row*x+starting));
          circle(src,Point(x_pos, y_pos+row*x),x,1);
          circle(source,Point(x_pos +hor_starting, 
                                  y_pos+row*x+starting),x,1);

          x_pos=0;y_pos=0;
          for (int k=0;k<c2.size();k++){
            x_pos += c2[k].x;
            y_pos += c2[k].y;
          }
          x_pos /= c2.size();
          y_pos /= c2.size();

          rod_points_subsampled_rodF.push_back(Point(x_pos, y_pos+row*x));
          rod_points_subsampled_picF.push_back(Point(x_pos+hor_starting,
                                                 y_pos+row*x+starting));

          circle(src,Point(x_pos, y_pos+row*x),x,1);
          circle(source,Point(x_pos+hor_starting, 
                                  y_pos+row*x+starting),x,1);
        }
          
      }
          
      else {*/
        // 1 cluster
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
        
      circle(vert_src,Point(x_pos, y_pos+row*x),x+3,1);
      circle(source,Point(x_pos +hor_starting,
                                         y_pos+row*x+starting),x+3,1);
                                         
      //}
    }
  }

///////////////////////////////////////////////////////////////////

  // show the ROI on the original image
  rectangle(source,Point(hor_starting,starting), 
                            Point(hor_ending,height+starting), 1);
                //namedWindow( "ROI", 1000 );
                //imshow( "ROI", flip_270(source) );
                  
  //save pnts+ROI over source threshold for troubleshooting
  std::string file_write = std::string(file_name);
  size_t lastIndex = file_write.find_last_of(".");
  std::string fileName = file_write.substr(0,lastIndex);
  std::string Tpoint = fileName;
  fileName.append("ROIandPnts");
  fileName.append(".jpg");
  imwrite(fileName, flip_270(source), (std::vector<int>){CV_IMWRITE_JPEG_QUALITY,0});
  /*
  time_t now = time(0);
  char* dt = ctime(&now);
  
  std::fstream myfile;
  myfile.open("rodPoints.txt", std::ios::in | std::ios::out | std::ios::ate);

  myfile << Tpoint;
  myfile << " Date: ";
  myfile << dt;
  for(int h=0;h<rod_points_subsampled_picF.size();h++){
    myfile << rod_points_subsampled_picF[h];
    myfile << "\n";
  }
  
  myfile.close();
  */

                // show the rod with ROI and points 
                //imshow( window_name2, vert_src );
  
  // export image with just rod pixels                          
  Mat ROD(source_rod.rows, source_rod.cols, CV_32F);
  Mat ROD_Isolated;
  ROD.setTo(255);        
  Rect RODSpace(hor_starting, starting, hor_ending-hor_starting, height);
  ROD_Isolated = source_rod(RODSpace);
  ROD_Isolated.copyTo(ROD.colRange(hor_starting, hor_ending).rowRange(starting, starting+height));
  
  std::string fileNameOutput = Tpoint;
  fileNameOutput.append("Isolated");
  fileNameOutput.append(".bmp");
  imwrite(fileNameOutput, flip_270(ROD));
}


int main( int argc, char** argv )
{
  Mat src; Mat src_gray; Mat src_thresh_gray;
  Mat seg_rod;

  src = imread(argv[1], 1 );

  ///flip sideways to work
  src=flip_90(src);

  /// Convert the image to Gray
  cvtColor( src, src_gray, CV_BGR2GRAY );
                //imwrite("gray_scaled_im.bmp", src_gray);

  /// Create a window to display results
  char* window_name = "Threshold Robot";
  char* window_name2 = "Segment Out Robot";
                //namedWindow( window_name, 1000 );
                //namedWindow( window_name2, 1000 );

  /// threshold the grey scaled image
  src_thresh_gray = Threshold( src_gray, window_name );
                //imwrite("thresh_im.bmp", src_thresh_gray);

  /// Loop over each vertical, % above N, is robot
  while (true)
  {
    try
    {
      Find_Rod( src_thresh_gray, window_name2, argv[1] );
      break;
    }
    catch(std::exception& e)
    {
      std::cout << "try again" << std::endl;
    }
  }
  std::cout << "actual finish" << std::endl;
        
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

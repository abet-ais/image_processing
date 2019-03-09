/**********************************************************************
   File    r_y_detect.cpp
   Author  Takahiro Yamazaki
   Environment    ROS_kinetic
   OS       Ubuntu 16.04 LTS
   StartDay 2019/3/7
   FinishDay 2019/3/7
**********************************************************************/
/**********************************************************************
   Problems to be fixed
    -
**********************************************************************/
/**********************************************************************
   Include Libraries
**********************************************************************/
//include
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>

#include <vector>
#include <numeric>
/**********************************************************************
   Declare variables
**********************************************************************/
//define
#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480

/**********************************************************************
   Globle
**********************************************************************/
/**********************************************************************
   Proto_type_Declare functions
**********************************************************************/
//prototype declaration
void image_detect(const sensor_msgs::ImageConstPtr& rgb_image);
/**********************************************************************
   Main
**********************************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "image_reading");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_rgb_image;

  sub_rgb_image = it.subscribe("/grobal/image_raw",1,&image_detect);

  cv::namedWindow("color_raw", CV_WINDOW_NORMAL);
  cv::resizeWindow("color_raw", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("color_gray", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_gray", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("color_b_w", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_b_w", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("canny_image", CV_WINDOW_NORMAL);
  // cv::resizeWindow("canny_image", WINDOW_WIDTH, WINDOW_HEIGHT);

  ros::spin();
  return 0;
}
/**********************************************************************
   Functions
**********************************************************************/
void image_detect(const sensor_msgs::ImageConstPtr& rgb_image){  //callback関数

  /*--- カメラのデータをopencvで扱える型に変更 ---*/
  cv_bridge::CvImagePtr cv_rgb;
  try{
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat color_raw  = cv_rgb->image;
  std::vector<float> x_yellow;
  std::vector<float> y_yellow;
  std::vector<float> x_red;
  std::vector<float> y_red;
  bool is_start = 0;
  int x_sum = 0;                //x座標合計
  int y_sum = 0;                //y座標合計
  float x_ave = 0.0;            //x座標平均
  float y_ave = 0.0;            //y座標平均
  int blue_channel = 0;
  int green_channel = 1;
  int red_channel = 2;
  int blue_threshold_yellow = 100;
  int green_threshold_yellow = 170;
  int red_threshold_yellow = 170;
  int blue_threshold_red = 100;
  int green_threshold_red = 100;
  int red_threshold_red = 170;


  for(int y=0; y<color_raw.rows; ++y){
    for(int x=0; x<color_raw.cols; ++x){
        for(int c = 0; c < color_raw.channels(); ++c){
          if( color_raw.data[y*color_raw.step + x*color_raw.elemSize() + red_channel ] > red_threshold_yellow && color_raw.data[y*color_raw.step + x*color_raw.elemSize() + green_channel ] > green_threshold_yellow && color_raw.data[y*color_raw.step + x*color_raw.elemSize() + blue_channel ] < blue_threshold_yellow){
            x_yellow.push_back(x);
            y_yellow.push_back(y);
          }//if yellow detect
          if( color_raw.data[y*color_raw.step + x*color_raw.elemSize() + red_channel ] > red_threshold_red && color_raw.data[y*color_raw.step + x*color_raw.elemSize() + green_channel ] < green_threshold_red){
            x_red.push_back(x);
            y_red.push_back(y);
          }//if red detect
    		}//for channels
      }// for cols
    }//for rows

    // x_sum = std::accumulate(x_yellow.begin(),x_yellow.end(),0);
    // y_sum = std::accumulate(y_yellow.begin(),y_yellow.end(),0);
    // x_ave = (float)x_sum / x_yellow.size();
    // y_ave = (float)y_sum / y_yellow.size();
    //
    //
    // cv::circle(color_raw,cv::Point(x_ave,y_ave),10,cv::Scalar(0,0,255),-1);
    if( !y_yellow.empty() ){
      float y_min_yellow = *std::max_element(y_yellow.begin(), y_yellow.end());
          cv::line( color_raw,cv::Point(0,y_min_yellow),cv::Point(WINDOW_WIDTH,y_min_yellow), cv::Scalar(255,255,0), 3, 8 );
    }
    //cv::circle(color_raw,cv::Point(x_ave,y_ave),10,cv::Scalar(0,0,255),-1);
    if( !y_red.empty() ){
      float y_min_red = *std::max_element(y_red.begin(), y_red.end());
          cv::line( color_raw,cv::Point(0,y_min_red),cv::Point(WINDOW_WIDTH,y_min_red), cv::Scalar(0,0,255), 3, 8 );
    }
    x_yellow.clear();      //vectorの中身を消去
    y_yellow.clear();
    x_red.clear();      //vectorの中身を消去
    y_red.clear();

    //printf("%f %f \n",x_ave,y_ave );

    /*----- 表示 -----*/
    cv::imshow("color_raw",color_raw);
    // cv::imshow("color_gray",color_gray);
    // cv::imshow("color_b_w",b_w_color);
    // cv::imshow("canny_image",canny_image);
    cv::waitKey(1);
}

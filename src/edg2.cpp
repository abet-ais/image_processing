/**********************************************************************
   File    edg2.cpp
   Author  Takahiro Yamazaki
   Environment    ROS_kinetic
   OS       Ubuntu 16.04 LTS
   StartDay 2019/3/3
**********************************************************************/
/**********************************************************************
   Problems to be fixed
    -
**********************************************************************/
/**********************************************************************
   Include Libraries
**********************************************************************/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>

// ros
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

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
/**********************************************************************
   Declare variables
**********************************************************************/
#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
/**********************************************************************
   Globle
**********************************************************************/
/**********************************************************************
   Proto_type_Declare functions
**********************************************************************/
void imageCb(const sensor_msgs::ImageConstPtr& rgb_image);
/**********************************************************************
   Main
**********************************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "image_reading");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_rgb_image;

  sub_rgb_image = it.subscribe("/grobal/image_raw",1,&imageCb);

  cv::namedWindow("color_raw", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("color_raw", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更
  cv::namedWindow("color_gray", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("color_gray", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更
  cv::namedWindow("color_b_w", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("color_b_w", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更
  cv::namedWindow("temp_dst", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("temp_dst", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更

  ros::spin();
  return 0;

}
/**********************************************************************
   Functions
**********************************************************************/
void imageCb(const sensor_msgs::ImageConstPtr& rgb_image){

  cv_bridge::CvImagePtr cv_rgb;
  try{
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat color_raw  = cv_rgb->image;
  cv::Mat color_gray;
  cv::Mat b_w_color;
  cv::Mat temp_dst;


  cvtColor(color_raw, color_gray,CV_RGB2GRAY);                 //grayスケール変換
  threshold(color_gray,b_w_color,160,255,cv::THRESH_BINARY);   //2値化
  cv::Canny(b_w_color, temp_dst, 50, 200, 3);

  cv::imshow("color_raw",color_raw);
  cv::imshow("color_gray",color_gray);
  cv::imshow("color_b_w",b_w_color);
  cv::imshow("temp_dst",temp_dst);

  cv::waitKey(1);
}

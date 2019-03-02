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

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720


void imageCb(const sensor_msgs::ImageConstPtr& rgb_image);

void imageCb(const sensor_msgs::ImageConstPtr& rgb_image){

  cv_bridge::CvImagePtr cv_rgb;
  try{
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Mat> sources;
  
  cv::Mat color_raw  = cv_rgb->image;
  cv::Mat color_gray;
  sources.push_back(color_raw);

  cvtColor(color_raw, color_gray,CV_RGB2GRAY);
  
  cv::imshow("color_raw",color_raw);
  cv::imshow("color_gray",color_gray);
  cv::waitKey(1);
}


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

  
  ros::spin();
  return 0;

}

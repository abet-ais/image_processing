/**********************************************************************
   File    yel_red_mouse_touch.cpp
   Author  abe
   Environment    ROS_kinetic
   OS       Ubuntu 16.04 LTS
   StartDay 2019/3/11
   FinishDay 2019/3/11
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <numeric>


/**********************************************************************
   Declare variables
**********************************************************************/
#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480


/**********************************************************************
   Globle
**********************************************************************/
cv::Mat color_raw;
cv::Mat depth_raw;
cv::Mat display_color;
cv::Mat display_depth;


/**********************************************************************
   Proto_type_Declare functions
**********************************************************************/
void image_detect(const sensor_msgs::ImageConstPtr& rgb_image,const sensor_msgs::ImageConstPtr& depth_image);
void on_mouse(int event, int x, int y, int flags, void* param );

/**********************************************************************
   Typedef
**********************************************************************/
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;


/**********************************************************************
   Main
**********************************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "image_reading");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_image(nh,"/camera/rgb/image_raw",1);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_image(nh,"/camera/depth/image_raw",1);
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
  sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(50);
  sync->connectInput(sub_rgb_image, sub_depth_image);
  sync->registerCallback(image_detect);

  cv::namedWindow("display_color", CV_WINDOW_NORMAL);
  cv::resizeWindow("display_color", WINDOW_WIDTH, WINDOW_HEIGHT);

  cvSetMouseCallback("display_color", on_mouse, 0);
  
  ros::spin();
  return 0;
}


/**********************************************************************
   Functions
**********************************************************************/
void image_detect(const sensor_msgs::ImageConstPtr& rgb_image,const sensor_msgs::ImageConstPtr& depth_image){

  /*--- カメラのデータをopencvで扱える型に変更 ---*/
  cv_bridge::CvImagePtr cv_rgb;
  try{
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  color_raw  = cv_rgb->image;
  display_color = color_raw.clone();
  
  /*--- カメラdepthデータをopencvで扱える型に変更 ---*/
  cv_bridge::CvImagePtr cv_depth;
  try{
    cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s",e.what());
    return;
  }
  depth_raw = cv_depth->image;

  cv::imshow("display_color",display_color);
  cv::waitKey(1);
}


void on_mouse(int event, int x, int y, int flags, void* param ){
  switch( event ){
  case CV_EVENT_MOUSEMOVE:
    {
      int b = display_color.at<cv::Vec3b>(y, x)[0];
      int g = display_color.at<cv::Vec3b>(y, x)[1];
      int r = display_color.at<cv::Vec3b>(y, x)[2];
      std::cout <<"b:"<<b << " " <<"g:"<< g << " "<<"r:"<< r << std::endl;
    }
  }
}

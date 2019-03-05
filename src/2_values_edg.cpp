/**********************************************************************
   File    2_values_edg.cpp
   Author  Takahiro Yamazaki
   Environment    ROS_kinetic
   OS       Ubuntu 16.04 LTS
   StartDay 2019/3/4
   FinishDay 2019/3/
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
  cv::namedWindow("color_bw", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("color_bw", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更
  cv::namedWindow("dst", CV_WINDOW_NORMAL);            //ウィンドウに名前をつける
  cv::resizeWindow("dst", WINDOW_WIDTH, WINDOW_HEIGHT);//ウィンドウのサイズを変更

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
  cv::Mat color_bw;
  cv::Mat dst = cv::Mat::zeros( 480,640,CV_8UC3 );

  cvtColor(color_raw, color_gray,CV_RGB2GRAY);                 //grayスケール変換
  threshold(color_gray,color_bw,160,255,cv::THRESH_BINARY);   //2値化

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours( color_bw, contours, hierarchy,CV_CHAIN_APPROX_TC89_L1, CV_CHAIN_APPROX_SIMPLE );

  int idx = 0;
  for( ; idx >= 0; idx = hierarchy[idx][0] ){
        cv::Scalar color( 0,255,0 );
        drawContours( dst, contours, idx, color, 3, 8, hierarchy );
      }

      // for(int i =0 ;i < 3;i++){
      //   for(int j =0;j <3;j++){
      //     printf("contours:: %f\r\n",contours[i][j] );
      //   }
      //   printf("\r\n");
      // }

      // printf("%f\n",contours[0][0] );
      // printf("%f\n",contours[0][1] );
      // printf("%f\n",contours[0][2] );
      // printf("%f\n",contours[0][3] );
      // printf("%f\n",contours[1][0] );
      // printf("%f\n",contours[1][1] );
      // printf("%f\n",contours[1][2] );
      // printf("%f\n",contours[2][0] );
      // printf("%f\n",contours[2][1] );
      // printf("%f\n",contours[2][2] );
      // printf("%f\n",contours[3][0] );
      // printf("%f\n",contours[3][1] );
      // printf("%f\n",contours[3][2] );

      printf("%f\n",contours[0] );
      printf("%f\n",contours[1] );
      printf("%f\n",contours[2] );
      printf("%f\n",contours[3] );

      cv::imshow( "color_raw",color_raw );
      cv::imshow( "color_bw",color_bw );
      cv::imshow( "dst",dst );

      cv::waitKey(1);
}

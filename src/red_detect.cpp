/**********************************************************************
   File    red_detect.cpp
   Author  Takahiro Yamazaki
   Environment    ROS_kinetic
   OS       Ubuntu 16.04 LTS
   StartDay 2019/3/6
   FinishDay 2019/3/6
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
//void imageCb(const sensor_msgs::ImageConstPtr& rgb_image);
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
   Functions 0
**********************************************************************/
//void imageCb(const sensor_msgs::ImageConstPtr& rgb_image){  //callback関数

  /*--- カメラのデータをopencvで扱える型に変更 ---*/
  // cv_bridge::CvImagePtr cv_rgb;
  // try{
  //   cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  // } catch(cv_bridge::Exception& e){
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }
  //
  // cv::Mat color_raw  = cv_rgb->image;
  // cv::Mat color_gray;
  // cv::Mat b_w_color;
  // cv::Mat canny_image;
  // std::vector<cv::Mat> sources;
  // sources.push_back(color_raw);
  //
  // /*----- 画像処理 -----*/
  // cvtColor(color_raw, color_gray,CV_RGB2GRAY);                 //grayスケール変換
  // threshold(color_gray,b_w_color,230,255,cv::THRESH_BINARY);   //2値化
  // Canny(b_w_color, canny_image,1,0);
  //
  // /*----- 最小二乗法による白線線形回帰 -----*/
  // std::vector<float> x_co;      //x座標
  // std::vector<float> y_co;      //y座標
  // int x_sum = 0;                //x座標合計
  // int y_sum = 0;                //y座標合計
  // float x_ave = 0.0;            //x座標平均
  // float y_ave = 0.0;            //y座標平均
  // std::vector<float> x_dif;     //(x_co - x_ave)
  // std::vector<float> y_dif;     //(y_co - y_ave)
  // float Sxy = 0.0;              //xとyの共分散
  // std::vector<float> Sxy_dif;   //(x_co - x_ave)*(y_co - y_ave)
  // float Sxy_sum = 0.0;          //Sxy_difの和
  // float Syy = 0.0;              //y分散
  // std::vector<float> Syy_dif_2; //pow((y_co - y_ave),2)
  // float Syy_sum = 0.0;          //pow((y_co - y_ave),2)の和
  //
  // float beta = 0.0;             //線形回帰 x = alpha + beta*y
  // float alpha= 0.0;
  //
  // for(int y=0; y<b_w_color.rows; ++y){
  //   for(int x=0; x<b_w_color.cols; ++x){
  //     if( b_w_color.data[y*b_w_color.step + x*b_w_color.elemSize()] == 255){ //白の時
  // 	x_co.push_back(x);
//y_co.push_back(y);
//       }
//     }
//   }
//
//   x_sum = std::accumulate(x_co.begin(),x_co.end(),0);
//   y_sum = std::accumulate(y_co.begin(),y_co.end(),0);
//   x_ave = (float)x_sum / x_co.size();
//   y_ave = (float)y_sum / y_co.size();
//
//   for(int i=0; i<x_co.size(); i++){
//     x_dif.push_back(x_co[i]-x_ave);
//     y_dif.push_back(y_co[i]-y_ave);
//     Syy_dif_2.push_back( std::pow(y_co[i]-y_ave,2) );
//   }
//
//   for(int i=0; i<x_dif.size(); i++){
//     Sxy_dif.push_back(x_dif[i]*y_dif[i]);
//   }
//
//   Sxy_sum = std::accumulate(Sxy_dif.begin(),Sxy_dif.end(),0);
//   Sxy = Sxy_sum / Sxy_dif.size();
//
//   Syy_sum = std::accumulate(Syy_dif_2.begin(),Syy_dif_2.end(),0);
//   Syy = Syy_sum / Syy_dif_2.size();
//
//   beta = Sxy / Syy;
//   alpha= x_ave - beta*y_ave;
//
//   x_co.clear();      //vectorの中身を消去
//   y_co.clear();
//   x_dif.clear();
//   y_dif.clear();
//   Sxy_dif.clear();
//   Syy_dif_2.clear();
//   sources.clear();
//
//   /*----- 点の描画 -----*/
//   float x = 0;
//   for(int y=0; y<b_w_color.cols; ++y){
//     x = alpha + beta*y;
//     cv::circle(color_raw,cv::Point((int)x,y),4,cv::Scalar(0));
//   }
//
//   /*----- 白線とのズレをわかりやすくするために点で軸を描画 -----*/
//   cv::Point origin;
//   origin.x = WINDOW_WIDTH/2;
//   origin.y = 300;
//   cv::line(color_raw,cv::Point(WINDOW_WIDTH/2,0),cv::Point(WINDOW_WIDTH/2,WINDOW_HEIGHT),cv::Scalar(100,100,100),4);
//   cv::line(color_raw,cv::Point(0,origin.y),cv::Point(WINDOW_WIDTH,origin.y),cv::Scalar(100,100,100),4);
//   cv::circle(color_raw,origin,10,cv::Scalar(100,100,100),4);
//
//   cv::circle(color_raw,cv::Point(alpha + beta*origin.y,origin.y),5,cv::Scalar(0,0,255),-1);
//
//
//   /*----- 中心ズレと回転角度 -----*/
//   float x_ = alpha - (alpha+beta*origin.y);  //y=0の時の
//   float y_ = (float)origin.y;
//   float theta = (180/M_PI)*atan(x_/y_);
//   float dif_origin_x = (alpha+beta*origin.y) - origin.x;
//   std::cout << "dif_origin_x: " << dif_origin_x << "  " << "theta: " << theta << std::endl;
//
//   cv::putText(color_raw,"dif(x)",cv::Point(10,50),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
//   cv::putText(color_raw,std::to_string(dif_origin_x),cv::Point(100,50),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
//   cv::putText(color_raw,"theta",cv::Point(10,100),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
//   cv::putText(color_raw,std::to_string(theta),cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
//
//   /*----- 表示 -----*/
//   cv::imshow("color_raw",color_raw);
//   // cv::imshow("color_gray",color_gray);
//   // cv::imshow("color_b_w",b_w_color);
//   // cv::imshow("canny_image",canny_image);
//   cv::waitKey(1);
// }
/**********************************************************************
   Functions 1
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
  std::vector<float> x_p;
  std::vector<float> y_p;
  bool is_start = 0;
  int x_sum = 0;                //x座標合計
  int y_sum = 0;                //y座標合計
  float x_ave = 0.0;            //x座標平均
  float y_ave = 0.0;            //y座標平均
  int blue_channel = 0;
  int green_channel = 1;
  int red_channel = 2;
  int blue_threshold = 100;
  int green_threshold = 100;
  int red_threshold = 170;


  for(int y=0; y<color_raw.rows; ++y){
    for(int x=0; x<color_raw.cols; ++x){
        for(int c = 0; c < color_raw.channels(); ++c){
          if( color_raw.data[y*color_raw.step + x*color_raw.elemSize() + red_channel ] > red_threshold && color_raw.data[y*color_raw.step + x*color_raw.elemSize() + green_channel ] < green_threshold){
            x_p.push_back(x);
            y_p.push_back(y);
          }//if color detect
    		}//for channels
      }// for cols
    }//for rows

    x_sum = std::accumulate(x_p.begin(),x_p.end(),0);
    y_sum = std::accumulate(y_p.begin(),y_p.end(),0);
    x_ave = (float)x_sum / x_p.size();
    y_ave = (float)y_sum / y_p.size();


    cv::circle(color_raw,cv::Point(x_ave,y_ave),10,cv::Scalar(0,0,255),-1);
    if( !y_p.empty() ){
      float y_min = *std::max_element(y_p.begin(), y_p.end());
          cv::line( color_raw,cv::Point(0,y_min),cv::Point(WINDOW_WIDTH,y_min), cv::Scalar(255,0,0), 3, 8 );
    }

    x_p.clear();      //vectorの中身を消去
    y_p.clear();

    printf("%f %f \n",x_ave,y_ave );

    /*----- 表示 -----*/
    cv::imshow("color_raw",color_raw);
    // cv::imshow("color_gray",color_gray);
    // cv::imshow("color_b_w",b_w_color);
    // cv::imshow("canny_image",canny_image);
    cv::waitKey(1);
}

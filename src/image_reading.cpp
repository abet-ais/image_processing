////////////////////////////////////////////////////////////////
//
//作成日：2019/3/5
//作成者：阿部
//
//目的：白線検出から最小二乗法を利用した線形回帰
//方法：生データ　→　グレースケール　→　２値化　→　エッジ検出(canny)
//     →エッジとして出た白点の数を数え，分布を見る　→　最小二乗法により線形回帰
//
////////////////////////////////////////////////////////////////

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


//define
#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480

//prototype declaration
void imageCb(const sensor_msgs::ImageConstPtr& rgb_image);



void imageCb(const sensor_msgs::ImageConstPtr& rgb_image){  //callback関数

  /*--- カメラのデータをopencvで扱える型に変更 ---*/
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
  cv::Mat canny_image;
  std::vector<cv::Mat> sources;
  sources.push_back(color_raw);
  
  /*----- 画像処理 -----*/
  cvtColor(color_raw, color_gray,CV_RGB2GRAY);                 //grayスケール変換
  threshold(color_gray,b_w_color,230,255,cv::THRESH_BINARY);   //2値化
  Canny(b_w_color, canny_image,1,0);

  /*----- 最小二乗法による白線線形回帰 -----*/
  std::vector<float> x_co;      //x座標
  std::vector<float> y_co;      //y座標
  int x_sum = 0;                //x座標合計
  int y_sum = 0;                //y座標合計
  float x_ave = 0.0;            //x座標平均
  float y_ave = 0.0;            //y座標平均
  std::vector<float> x_dif;     //(x_co - x_ave)
  std::vector<float> y_dif;     //(y_co - y_ave)
  float Sxy = 0.0;              //xとyの共分散
  std::vector<float> Sxy_dif;   //(x_co - x_ave)*(y_co - y_ave)
  float Sxy_sum = 0.0;          //Sxy_difの和
  float Syy = 0.0;              //y分散
  std::vector<float> Syy_dif_2; //pow((y_co - y_ave),2)
  float Syy_sum = 0.0;          //pow((y_co - y_ave),2)の和
			
  float beta = 0.0;             //線形回帰 x = alpha + beta*y
  float alpha= 0.0;
  
  for(int y=0; y<b_w_color.rows; ++y){
    for(int x=0; x<b_w_color.cols; ++x){
      if( b_w_color.data[y*b_w_color.step + x*b_w_color.elemSize()] == 255){ //白の時
  	x_co.push_back(x);
 	y_co.push_back(y);
      }
    }
  }

  x_sum = std::accumulate(x_co.begin(),x_co.end(),0);
  y_sum = std::accumulate(y_co.begin(),y_co.end(),0);
  x_ave = (float)x_sum / x_co.size();
  y_ave = (float)y_sum / y_co.size();

  for(int i=0; i<x_co.size(); i++){
    x_dif.push_back(x_co[i]-x_ave);
    y_dif.push_back(y_co[i]-y_ave);
    Syy_dif_2.push_back( std::pow(y_co[i]-y_ave,2) );
  }

  std::cout <<  x_ave << "  " << y_ave << std::endl;
  
  for(int i=0; i<x_dif.size(); i++){
    Sxy_dif.push_back(x_dif[i]*y_dif[i]);
  }

  Sxy_sum = std::accumulate(Sxy_dif.begin(),Sxy_dif.end(),0);
  Sxy = Sxy_sum / Sxy_dif.size();

  Syy_sum = std::accumulate(Syy_dif_2.begin(),Syy_dif_2.end(),0);
  Syy = Syy_sum / Syy_dif_2.size();

  beta = Sxy / Syy;
  alpha= x_ave - beta*y_ave;

  x_co.clear();      //vectorの中身を消去
  y_co.clear();
  x_dif.clear();
  y_dif.clear();
  Sxy_dif.clear();
  Syy_dif_2.clear();
  sources.clear();
  

  /*----- 点の描画 -----*/
  float x = 0;
  for(int y=0; y<b_w_color.cols; ++y){
    x = alpha + beta*y;
    cv::circle(color_raw,cv::Point((int)x,y),4,cv::Scalar(0));
  }

  /*----- 白線とのズレをわかりやすくするために軸を描画 -----*/
  cv::line(color_gray,cv::Point(WINDOW_WIDTH/2,0),cv::Point(WINDOW_WIDTH/2,WINDOW_HEIGHT),cv::Scalar(100,100,100),4);
  cv::line(color_gray,cv::Point(0,300),cv::Point(WINDOW_WIDTH,300),cv::Scalar(100,100,100),4);
  cv::circle(color_gray,cv::Point(WINDOW_WIDTH/2,300),10,cv::Scalar(100,100,100),4);


  /*----- 表示 -----*/
  cv::imshow("color_raw",color_raw);
  // cv::imshow("color_gray",color_gray);
  // cv::imshow("color_b_w",b_w_color);
  cv::imshow("canny_image",canny_image);
  cv::waitKey(1);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "image_reading");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_rgb_image;

  sub_rgb_image = it.subscribe("/grobal/image_raw",1,&imageCb);
  
  cv::namedWindow("color_raw", CV_WINDOW_NORMAL);            
  cv::resizeWindow("color_raw", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("color_gray", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_gray", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("color_b_w", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_b_w", WINDOW_WIDTH, WINDOW_HEIGHT);
  cv::namedWindow("canny_image", CV_WINDOW_NORMAL);
  cv::resizeWindow("canny_image", WINDOW_WIDTH, WINDOW_HEIGHT);
  
  ros::spin();
  return 0;
}

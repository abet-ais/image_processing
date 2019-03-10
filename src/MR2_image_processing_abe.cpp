/**********************************************************************
   File    MR2_image_processing_abe.cpp
   Author  Takahiro Yamazaki  and abe
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <numeric>


/**********************************************************************
   Declare variables
**********************************************************************/
//define
#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480
#define XTION_HEIGHT -0.473    //checker_boardからxtion原点までの高さ距離(外部パラメータ)
#define XTION_ROLL_RAD 0.411   //checker_boardからxtionのroll[rad](外部パラメータ)


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
void red_yellow_detection(const cv::Mat color_raw, const cv::Mat depth_raw);
void white_line_detection(const cv::Mat color_raw);


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

  // cv::namedWindow("color_raw", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_raw", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("color_gray", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_gray", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("color_b_w", CV_WINDOW_NORMAL);
  // cv::resizeWindow("color_b_w", WINDOW_WIDTH, WINDOW_HEIGHT);
  // cv::namedWindow("canny_image", CV_WINDOW_NORMAL);
  // cv::resizeWindow("canny_image", WINDOW_WIDTH, WINDOW_HEIGHT);
  cv::namedWindow("display_color", CV_WINDOW_NORMAL);
  cv::resizeWindow("display_color", WINDOW_WIDTH, WINDOW_HEIGHT);

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

  red_yellow_detection( color_raw, depth_raw ); //赤，黃検知
  white_line_detection( color_raw );            //白線検知
}


void red_yellow_detection(const cv::Mat color_raw_ ,const cv::Mat depth_raw_){ //赤色・黄色検出
  std::vector<float> x_yellow;
  std::vector<float> y_yellow;
  std::vector<float> x_red;
  std::vector<float> y_red;
  // bool is_start = 0;
  int blue_channel  = 0;
  int green_channel = 1;
  int red_channel   = 2;
  int blue_threshold_yellow = 100;
  int green_threshold_yellow = 170;
  int red_threshold_yellow = 170;
  int blue_threshold_red = 100;
  int green_threshold_red = 100;
  int red_threshold_red = 170;
  float yellow_depth = 0.0;   //黄色の座標の距離データ格納
  int y_max = 0;              //yが最大の時ものを格納
  int x_ymax= 0;              //yが最大の時のx座標を格納(xの最大ではない)
  float yellow_board_distance; //xtionから黄色の板までの距離

  for(int y=0; y<color_raw_.rows; ++y){
    for(int x=0; x<color_raw_.cols; ++x){
      for(int c = 0; c < color_raw_.channels(); ++c){
	int chanel_count = y*color_raw_.step+x*color_raw_.elemSize();
	if( (color_raw_.data[chanel_count + red_channel] > red_threshold_yellow)
	    && (color_raw_.data[chanel_count + green_channel] > green_threshold_yellow)
	    && (color_raw_.data[chanel_count + blue_channel] < blue_threshold_yellow) ){
	  // x_yellow.push_back(x);
	  y_yellow.push_back(y);
	  if(y_max < y){
	    y_max = y;
	    x_ymax= x;
	  }
	}//if yellow detect
	if( (color_raw_.data[chanel_count+ red_channel] > red_threshold_red)
	    && (color_raw_.data[chanel_count + green_channel] < green_threshold_red) ){
	  x_red.push_back(x);
	  y_red.push_back(y);
	}//if red detect
      }//for channels
    }// for cols
  }//for rows
  
  if( !y_yellow.empty() ){
    // float y_min_yellow = *std::max_element(y_yellow.begin(), y_yellow.end());
    yellow_depth = depth_raw_.at<cv::Vec3f>(y_max,x_ymax)[0]/1000;  //[mm]で出力のため[m]に変換
    cv::line( display_color,cv::Point(0,y_max),cv::Point(WINDOW_WIDTH,y_max), cv::Scalar(0,100,100), 3, 8 );
    cv::circle( display_color,cv::Point(x_ymax,y_max),5,cv::Scalar(0,100,100),-1);
  }
  //cv::circle(display_color,cv::Point(x_ave,y_ave),10,cv::Scalar(0,0,255),-1);
  if( !y_red.empty() ){
    float y_min_red = *std::max_element(y_red.begin(), y_red.end());
    cv::line(display_color,cv::Point(0,y_min_red),cv::Point(WINDOW_WIDTH,y_min_red), cv::Scalar(0,0,255), 3, 8 );
  }
  x_yellow.clear();      //vectorの中身を消去
  y_yellow.clear();
  x_red.clear();
  y_red.clear();

  yellow_board_distance = sqrt( pow(yellow_depth,2) - pow(XTION_HEIGHT,2));
  
  std::cout << "黄色までの距離[m]: " << yellow_board_distance << std::endl;
  //printf("%f %f \n",x_ave,y_ave );
}


void white_line_detection(const cv::Mat color_raw_){ //白線検出
  cv::Mat color_gray;
  cv::Mat b_w_color;
  cv::Mat canny_image;
  std::vector<cv::Mat> sources;
  sources.push_back(color_raw);

  /*----- 画像処理 -----*/
  cvtColor(color_raw_, color_gray,CV_RGB2GRAY);                 //grayスケール変換
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
    cv::circle(display_color,cv::Point((int)x,y),4,cv::Scalar(0));
  }

  /*----- 白線とのズレをわかりやすくするために点で軸を描画 -----*/
  cv::Point origin;
  origin.x = WINDOW_WIDTH/2;
  origin.y = 300;
  cv::line(display_color,cv::Point(WINDOW_WIDTH/2,0),cv::Point(WINDOW_WIDTH/2,WINDOW_HEIGHT),cv::Scalar(100,100,100),4);
  cv::line(display_color,cv::Point(0,origin.y),cv::Point(WINDOW_WIDTH,origin.y),cv::Scalar(100,100,100),4);
  cv::circle(display_color,origin,10,cv::Scalar(100,100,100),4);
  cv::circle(display_color,cv::Point(alpha + beta*origin.y,origin.y),5,cv::Scalar(0,0,255),-1);


  /*----- 中心ズレと回転角度 -----*/
  float x_ = alpha - (alpha+beta*origin.y);  //y=0の時の
  float y_ = (float)origin.y;
  float theta = (180/M_PI)*atan(x_/y_);
  float dif_origin_x = (alpha+beta*origin.y) - origin.x;
  // std::cout << "dif_origin_x: " << dif_origin_x << "  " << "theta: " << theta << std::endl;

  cv::putText(display_color,"dif(x)",cv::Point(10,50),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);
  cv::putText(display_color,std::to_string(dif_origin_x),cv::Point(100,50),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);
  cv::putText(display_color,"theta",cv::Point(10,100),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);
  cv::putText(display_color,std::to_string(theta),cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);

  cv::imshow("display_color",display_color);
  // cv::imshow("color_gray",color_gray);
  // cv::imshow("color_b_w",b_w_color);
  // cv::imshow("canny_image",canny_image);
  cv::waitKey(1);
}

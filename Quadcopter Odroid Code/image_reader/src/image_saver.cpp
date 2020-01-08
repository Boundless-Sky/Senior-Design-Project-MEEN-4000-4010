#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sstream> 
#include "std_msgs/String.h"
using namespace cv;
using namespace std;
int MinHb=78,MaxHb=124,MinSb=60,MaxSb=255,MinVb=60,MaxVb=255;//blue
int MinHo=10,MaxHo=30,MinSo=80,MaxSo=255,MinVo=80,MaxVo=255;//ora
int erosion_size=2;
static const std::string OPENCV_WINDOW = "Image window";
geometry_msgs::Point P1,P2;
int xxx = 100, yyy = 100;
Point window_b(xxx, yyy);
bool ifthing_b = true, ifthing_o = true, ifthing_b_last = false, ifthing_o_last = false;
int ii = 0;
double u1d,u2d,v1d,v2d;
int WIDTH = 640, HEIGHT = 480;
VideoCapture capture(0);
VideoWriter writer("/media/odroid/DAVID/top_video_CnoRC.avi",CV_FOURCC('M','J','P','G'),10.0,Size(640,480));
Mat img;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{ 
 img=cv_bridge::toCvShare(msg, "bgr8")->image;
 circle(img, cvPoint(u1d,v1d), 5, CV_RGB(0, 0, 0), 7, 8);
 circle(img, cvPoint(u2d,v2d), 5, CV_RGB(0, 255, 0), 7, 8);
cout<<u1d<<v1d<<u2d<<v2d<<endl;
//imshow("raw", img);
//waitKey(1);
 writer << img;
 waitKey(1); 
}
void image_d1_Callback(const geometry_msgs::Point& msg1)
{
 u1d=msg1.x;
 v1d=msg1.y;
//cout<<"!!!"<<endl;
}
void image_d2_Callback(const geometry_msgs::Point& msg)
{
 u2d=msg.x;                 
 v2d=msg.y;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver_ODROID");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
ros::Subscriber image_d1_sub = nh.subscribe("d_image1",1,image_d1_Callback);
	ros::Subscriber image_d2_sub = nh.subscribe("d_image2",1,image_d2_Callback);
  image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
  ros::Rate loop_rate(12.5);
Mat img;
sensor_msgs::ImagePtr msg;
  while (ros::ok()){
//int64 t1 = getTickCount();
	//bool bSuccess = cap.read(img);
 	//if (!bSuccess) break;


///////////////////////////////////////////////////////////////////
   


   	ros::spinOnce();
    	loop_rate.sleep();
//int64 t2 = getTickCount();
		//double T = (t2 - t1) / getTickFrequency();
		//cout << "T_save=" << T << endl;
  }
  return 0;
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
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
int WIDTH =1920 , HEIGHT = 1080; // HD camera: 1920 x 1080	 WEBCAM: 640 x 480 
// changing this seems to not do anything compared to image_total

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reader_ODROID");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pubi = it.advertise("camera/image", 1);
  ros::Rate loop_rate(12.5);
  VideoCapture cap(0); // VideoCapture cap(0) is default, VideoCapture cap(1) is the usb camera
cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH); //Width of the frames in the video stream.
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
Mat img;
sensor_msgs::ImagePtr msg;
  while (ros::ok()){
int64 t1 = getTickCount();
ifthing_b = true; ifthing_o = true;
	bool bSuccess = cap.read(img);
 	if (!bSuccess) break;
///////////////////////////////////////////////////////////////////
   	ros::spinOnce();
    	loop_rate.sleep();
if(!img.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      pubi.publish(msg);
int64 t2 = getTickCount();
		double T = (t2 - t1) / getTickFrequency();
		cout << "T_reader=" << T << endl;
    }
  }
  return 0;
}


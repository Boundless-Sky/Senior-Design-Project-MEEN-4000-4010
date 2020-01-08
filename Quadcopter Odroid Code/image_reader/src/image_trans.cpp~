#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> 
#include "std_msgs/String.h"
using namespace cv;
using namespace std;
int MinHb=78,MaxHb=124,MinSb=60,MaxSb=255,MinVb=60,MaxVb=255;//blue
int MinHo=10,MaxHo=30,MinSo=80,MaxSo=255,MinVo=80,MaxVo=255;//ora
int erosion_size=2;
static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_trans");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Rate loop_rate(50);
  VideoCapture cap(0); 
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280); 
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
sensor_msgs::ImagePtr msg;
VideoCapture capture(0);
VideoWriter writer("11.avi",CV_FOURCC('M','J','P','G'),10.0,Size(1280,720));
//cap.set(CV_CAP_PROP_SETTINGS,1);
  while (ros::ok()){
	int64 t1 = getTickCount();
	bool ifthing_b = true, ifthing_o = true;
	Mat img,img_HSV,img_blu,img_blu2,img_ora,img_ora2;
	double largest_area=0;int largest_contour_index=0;
	vector<vector<Point> > contours_b,contours_o;
  	vector<Vec4i> hierarchy_b,hierarchy_o;
	bool bSuccess = cap.read(img);
 	if (!bSuccess) break;
///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
if(!img.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      pub.publish(msg);
imshow("raw", img); 
    }


//////////////////////////////////////////////////////////////////	
	//writer << img;
    	waitKey(1);  
   	ros::spinOnce();
    	loop_rate.sleep();
int64 t2 = getTickCount();
		double T = (t2 - t1) / getTickFrequency();
		cout << T <<"s"<< endl;
  }
  return 0;
}

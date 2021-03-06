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
int MinHr1=160,MaxHr1=179;
int MinHr2=0,MaxHr2=25,MinSr=80,MaxSr=150,MinVr=100,MaxVr=255;//red
int MinHb=50,MaxHb=110,MinSb=20,MaxSb=200,MinVb=140,MaxVb=255;//blue
int MinHo=0,MaxHo=25,MinSo=30,MaxSo=80,MinVo=100,MaxVo=250;//ora
int erosion_size=1;
static const std::string OPENCV_WINDOW = "Image window";
geometry_msgs::Point P1,P2;
int xxx = 100, yyy = 100;
Point window_b(xxx, yyy);
bool ifthing_b = true, ifthing_o = true, ifthing_b_last = false, ifthing_o_last = false;
int ii = 0;
int WIDTH = 640, HEIGHT = 480;
Point img_process(Mat img, int color, image_transport::Publisher pub_HSVb,image_transport::Publisher pub_HSVo)
{
sensor_msgs::ImagePtr msgHSV_o,msgHSV_b;
	Mat img_HSV, img_gra,img_gra1,img_gra2, img2; RotatedRect minEllipse; int largest_contour_index = 0; double largest_area = 0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cvtColor(img, img_HSV, CV_BGR2HSV);
	if (color == 0)
		inRange(img_HSV, Scalar(MinHb, MinSb, MinVb), Scalar(MaxHb + 1, MaxSb + 1, MaxVb + 1), img_gra);
	else
{
inRange(img_HSV, Scalar(MinHr1, MinSr, MinVr), Scalar(MaxHr1 + 1, MaxSr + 1, MaxVr + 1), img_gra1);
inRange(img_HSV, Scalar(MinHr2, MinSr, MinVr), Scalar(MaxHr2 + 1, MaxSr + 1, MaxVr + 1), img_gra2);
img_gra=img_gra1+img_gra2;
}
		//inRange(img_HSV, Scalar(MinHo, MinSo, MinVo), Scalar(MaxHo + 1, MaxSo + 1, MaxVo + 1), img_gra);
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	erode(img_gra, img2, element);
	dilate(img2, img_gra, element);
dilate(img_gra, img_gra, element);
	if (color == 0)
	{
		//imshow("imgb", img_gra);
			msgHSV_b = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gra).toImageMsg();
		         pub_HSVb.publish(msgHSV_b);
		//waitKey(1);
	}
	else
	{
			msgHSV_o = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gra).toImageMsg();
		         pub_HSVo.publish(msgHSV_o);
		//imshow("imgo", img_gra);
		//waitKey(1);
	}
	findContours(img_gra, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++) {
		double a = contourArea(contours[i], false);
		if (a > largest_area){
			largest_area = a;
			largest_contour_index = i;
		}
	}
	if (color == 0)
	{
		if (contours.size() == 0)
			ifthing_b = false;
		if (ifthing_b)
		{
			if (contours[largest_contour_index].size() <= 5)
				ifthing_b = false;
			if (ifthing_b)
			{
				minEllipse = fitEllipse(Mat(contours[largest_contour_index]));
				return minEllipse.center;
			}
		}
	}
	else
	{
		if (contours.size() == 0)
			ifthing_o = false;
		if (ifthing_o)
		{
			if (contours[largest_contour_index].size() <= 5)
				ifthing_o = false;
			if (ifthing_o)
			{
				minEllipse = fitEllipse(Mat(contours[largest_contour_index]));
				return minEllipse.center;
			}
		}
		if (!ifthing_o || !ifthing_b)
		{
			return Point(0,0);
		}
	}
	return minEllipse.center;
}
Rect find_win(Point center)
{
	Point lt;
	if (center.x <= window_b.x / 2)
		lt.x = 1;
	else if (center.x >= WIDTH - window_b.x / 2)
		lt.x = WIDTH - window_b.x;
	else
		lt.x = center.x - window_b.x / 2;
	if (center.y <= window_b.y / 2)
		lt.y = 1;
	else if (center.y >= HEIGHT - window_b.y / 2)
		lt.y = HEIGHT - window_b.y;
	else
		lt.y = center.y - window_b.y / 2;
	Rect roi(lt.x, lt.y, window_b.x, window_b.y);
	return roi;
}
int main(int argc, char** argv)
{
Mat img,img_HSV, img_blu, img_blu2, img_ora, img_ora2;
	Mat img_roi, img_HSV_roi, img_blu_roi, img_blu2_roi, img_ora_roi, img_ora2_roi;
	Point center_b = Point(500, 500); Point center_o = Point(500, 500); Point center_b_win = Point(500, 500); Point center_o_win = Point(500, 500);
	int64 tb1, tb2, tbb1, tbb2, to1, to2, too1, too2;
sensor_msgs::ImagePtr msg;
  ros::init(argc, argv, "image_processer_ODROID");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  ros::Publisher pub1 = n.advertise<geometry_msgs::Point>("image_point1", 1);
  ros::Publisher pub2 = n.advertise<geometry_msgs::Point>("image_point2", 1);
image_transport::Publisher pub = it.advertise("camera/image", 1);
image_transport::Publisher pub_HSVb = it.advertise("camera/image_HSVb", 1);
image_transport::Publisher pub_HSVo = it.advertise("camera/image_HSVo", 1);
  ros::Rate loop_rate(10);
  VideoCapture cap(0);
cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
  while (ros::ok()){
int64 t1 = getTickCount();
ifthing_b = true; ifthing_o = true;
	bool bSuccess = cap.read(img);
 	if (!bSuccess){
            cout<<cap.isOpened()<<endl<<endl;
	    cerr<<"Error Opening Camera"<<endl;
	    break;
	}
///////////////////////////////////////////////////////////////////
		if (ii == 0 || !ifthing_b_last)
		{
			 //tb1 = getTickCount();
			center_b = img_process(img, 0,pub_HSVb, pub_HSVo);
			if (ifthing_b)
			{
				circle(img, center_b, 5, CV_RGB(0, 0, 255), 2, 8);P1.x=center_b.x;P1.y=center_b.y;
			}
			 //tb2 = getTickCount();
			 //double Tb = (tb2 - tb1) / getTickFrequency();
			 //cout<< "TB =" << Tb << endl;
		}
		if (ii > 0 && ifthing_b_last)
		{
			 //tbb1 = getTickCount();
			Rect roi_b = find_win(center_b);
			Mat img_roib = img(roi_b);
			center_b_win = img_process(img_roib, 0,pub_HSVb, pub_HSVo);
			center_b = center_b_win + Point(roi_b.x,roi_b.y);
			if (ifthing_b)
			{
				circle(img, center_b, 5, CV_RGB(0, 0, 255), 2, 8);P1.x=center_b.x;P1.y=center_b.y;
			}
			 //tbb2 = getTickCount();
			 //double Tbb = (tbb2 - tbb1) / getTickFrequency();
			 //cout << "TBB=" << Tbb << endl;
		}
		if (ii == 0 || !ifthing_o_last)
		{
			//to1 = getTickCount();
			center_o = img_process(img, 1,pub_HSVb, pub_HSVo);
			if (ifthing_o)
			{
				circle(img, center_o, 5, CV_RGB(255, 0, 0), 2, 8);P2.x=center_o.x;P2.y=center_o.y;
			}
			//to2 = getTickCount();
			//double To = (to2 - to1) / getTickFrequency();
			//cout << "To =" << To << endl;
		}
		if (ii > 0 && ifthing_o_last)
		{
			//too1 = getTickCount();
			Rect roi_o = find_win(center_o);
			Mat img_roio = img(roi_o);
			center_o_win = img_process(img_roio, 1,pub_HSVb, pub_HSVo);
			center_o = center_o_win + Point(roi_o.x, roi_o.y);
			if (ifthing_o)
			{
				circle(img, center_o, 5, CV_RGB(255, 0, 0), 2, 8);P2.x=center_o.x;P2.y=center_o.y;
			}
			//too2 = getTickCount();
			//double Too = (too2 - too1) / getTickFrequency();
			//cout << "Too=" << Too << endl;
		}
		//////////////////////////////////////////////////////////////////
		//imshow("raw", img);
		//waitKey(1);  
		if(!img.empty()) 
		{
		         msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		         pub.publish(msg);
		}
   	ros::spinOnce();
    	loop_rate.sleep();
//cout << "bx=" << P2.x << endl;
//cout << "by=" << P2.y << endl;
// t1 = getTickCount();
int64 t2 = getTickCount();
		double T = (t2 - t1) / getTickFrequency();
		cout << "T_processer=" << T << endl;
P1.z=T;P2.z=T;
pub1.publish(P1); 
pub2.publish(P2); 
    
  }
  return 0;
}

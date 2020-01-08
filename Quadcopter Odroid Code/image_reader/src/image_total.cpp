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
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

using namespace cv;
using namespace std;

//these are now the magenta values
int MinHr1=162,MaxHr1=173;
int MinHr2=162,MaxHr2=173,MinSr=100,MaxSr=205,MinVr=100,MaxVr=170;//red

//these are now the cyan values
//int MinHb=70,MaxHb=104,MinSb=50,MaxSb=255,MinVb=244,MaxVb=255;//blue
int MinHb=83,MaxHb=115,MinSb=150,MaxSb=255,MinVb=182,MaxVb=255;//blue
//int MinHb=93,MaxHb=110,MinSb=82,MaxSb=160,MinVb=105,MaxVb=153;//blue

//this is not used in the code
int MinHo=145,MaxHo=160,MinSo=70,MaxSo=105,MinVo=152,MaxVo=180;//ora
int erosion_size=2;
static const std::string OPENCV_WINDOW = "Image window";

geometry_msgs::Point P1,P2;
//geometry_msgs::Quaternion centroid;
geometry_msgs::Vector3 centroid;
geometry_msgs::Vector3 T_lost_centroid;

int xxx = 150, yyy = 150;
Point window_b(xxx, yyy);
bool ifthing_b = true, ifthing_o = true, ifthing_b_last = false, ifthing_o_last = false;
int ii = 0;
//(4-12-2017)
//int WIDTH = 1920, HEIGHT = 1080; // HD camera: 1920 x 1080 16:9-> 1024 x 576	 WEBCAM: 640 x 480
//int WIDTH = 1024, HEIGHT = 576;
int WIDTH = 640, HEIGHT = 480;
int jcount=0; // iteration counter for data collection

Point img_process(Mat img, int color, image_transport::Publisher pub_HSVb,image_transport::Publisher pub_HSVo)
{ 
	sensor_msgs::ImagePtr msgHSV_o,msgHSV_b;
	Mat img_HSV, img_gra,img_gra2,img_gra1, img2; RotatedRect minEllipse; int largest_contour_index = 0;
	Mat img_HSVinit;
	double largest_area = 0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	// 4-12 
	//cvtColor(img, img_HSVinit, CV_BGR2HSV);
	cvtColor(img,img_HSV,CV_BGR2HSV);
	//img_HSV = img_HSVinit(Rect(640, 300,640,480));
        //(4-12-2017)
	//img_HSV = img_HSVinit(Rect(448, 252,1024,576));//take the 0, 0 of image to start at different location (cropping)
	if (color == 0)
		inRange(img_HSV, Scalar(MinHb, MinSb, MinVb), Scalar(MaxHb + 1, MaxSb + 1, MaxVb + 1), img_gra);
	else
    {
    	inRange(img_HSV, Scalar(MinHr1, MinSr, MinVr), Scalar(MaxHr1 + 1, MaxSr + 1, MaxVr + 1), img_gra1);
      inRange(img_HSV, Scalar(MinHr2, MinSr, MinVr), Scalar(MaxHr2 + 1, MaxSr + 1, MaxVr + 1), img_gra);
       img_gra=img_gra1+img_gra2;
    }
        
//	inRange(img_HSV, Scalar(MinHo, MinSo, MinVo), Scalar(MaxHo + 1, MaxSo + 1, MaxVo + 1), img_gra);
		Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
//	erode(img_gra, img2, element);
//	dilate(img2, img_gra, element);
//	dilate(img_gra, img_gra, element); 
       if (color == 0)
        {
//                msgHSV_b = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gra).toImageMsg();
//                         pub_HSVb.publish(msgHSV_b);
        }
        else
        {
//                msgHSV_o = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gra).toImageMsg();
//                         pub_HSVo.publish(msgHSV_o);
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
}//end image_process

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
	Point center_b = Point(500, 500); Point center_o = Point(500, 500); Point center_b_win = Point(500, 	500); 	
	Point center_o_win = Point(500, 500);
	int64 tb1, tb2, tbb1, tbb2, to1, to2, too1, too2;
	sensor_msgs::ImagePtr msg;
	ros::init(argc, argv, "image_processer_ODROID");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	ros::Publisher pub1 = n.advertise<geometry_msgs::Point>("image_point1", 1); //blue
	ros::Publisher pub2 = n.advertise<geometry_msgs::Point>("image_point2", 1); //red
	//ros::Publisher pub3 = n.advertise<geometry_msgs::Quaternion>("image_centroid", 1);
	ros::Publisher pub3 = n.advertise<geometry_msgs::Vector3>("image_centroid", 1);
	ros::Publisher pub4 = n.advertise<geometry_msgs::Vector3>("time_lost_centroid", 1);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	image_transport::Publisher pub_HSVb = it.advertise("camera/image_HSVb", 1);
	image_transport::Publisher pub_HSVo = it.advertise("camera/image_HSVo", 1);

	// 4-12-2017 ros::Rate loop_rate(10); //20 hz with roscore on odroid is working very well
	ros::Rate loop_rate(20);
	VideoCapture cap(0); // VideoCapture cap(0) is default, VideoCapture cap(1) is the usb camera
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	cap.set(CV_CAP_PROP_SETTINGS,1);
	int lastP1x [3] = {320, 320, 320}, lastP1y [3] = {240, 240, 240};
	int lastP2x, lastP2y;
	int n_c = 0;
	double T_found = 0.0; //time since the marker centroids were found

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
					circle(img, center_b, 5, CV_RGB(0, 0, 255), 2, 8);
					P1.x=center_b.x;P1.y=center_b.y;
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
					circle(img, center_b, 5, CV_RGB(0, 0, 255), 2, 8);
					P1.x=center_b.x;P1.y=center_b.y;
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
					circle(img, center_o, 5, CV_RGB(255, 0, 0), 2, 8);
					P2.x=center_o.x;P2.y=center_o.y;
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
					circle(img, center_o, 5, CV_RGB(255, 0, 0), 2, 8);
					P2.x=center_o.x;P2.y=center_o.y;
				}
				//too2 = getTickCount();
				//double Too = (too2 - too1) / getTickFrequency();
				//cout << "Too=" << Too << endl;
			}
			//////////////////////////////////////////////////////////////////
			//imshow("raw", img);
			//waitKey(1);  
		if(!img.empty()) {
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	    pub.publish(msg);
    }
   	ros::spinOnce();
    	
	// t1 = getTickCount();
	int64 t2 = getTickCount();
	double T = (t2 - t1) / getTickFrequency();

//Cutting off extreme outliers (sporadic data and if it cant find blue)
	//booleans for whether a centroid was correctly found
	bool centroids_found = true;

	if (n_c > 0 ){
 		if((!ifthing_b) ){
			P1.x = lastP1x[0];
			centroids_found = false;
 		}
		if((!ifthing_b) ){
			P1.y = lastP1y[0];
			centroids_found = false;
 		}
	//	if(P2.x > lastP2x + 200 || P2.x < lastP2x - 200 ){ //need to add !ifthing_o so it can check if 
		//	P2.x = lastP2x;	//it has found the color red...
			//centroids_found = false;
		//}
		//if(P2.y > lastP2y + 200 || P2.y < lastP2y - 200 ){
			//P2.y = lastP2y;
			//centroids_found = false;
 		//}
	}
	if(centroids_found){
		T_found = 0;
	}
	else{
		T_found += T;
	}

	++n_c;
	//lastP2x = P2.x;
	//lastP2y = P2.y;

	//find centroid
	double center_x = (P1.x + lastP1x[0] + lastP1x[1] + lastP1x[2])/4.0; //P1.x/2 + P2.x/2;
	double center_y = (P1.y + lastP1y[0] + lastP1y[1] + lastP1y[2])/4.0; //P1.y/2 + P2.y/2;
	centroid.x = center_x;
	centroid.y = center_y;

	//save new last values
	lastP1x[2] = lastP1x[1];
	lastP1x[1] = lastP1x[0];
	lastP1x[0] = P1.x;
	lastP1y[2] = lastP1y[1];
	lastP1y[1] = lastP1y[0];
	lastP1y[0] = P1.y;

	//find rotation (assuming the magenta is on the right when the robot has zero rotation)
	//note: values are negative to compensate for y being positive downwards
	double rot = 0;
	if(P2.x > P1.x){
		rot = -atan((P2.y - center_y)/(P2.x - center_x));
	}
	else{
		if(P2.y > P1.y){
			rot = -(atan((P1.y - center_y)/(P1.x - center_x)) + 3.14159);
	}
	else{
		rot = -(atan((P1.y - center_y)/(P1.x - center_x)) - 3.14159);
	}
	}
	
	//convert rotation to degrees
	rot = (rot/3.14159)*180;

	//centroid.z = rot; (THIS IS THE ORIGINAL)
	
	/*cout << "T_processer|" ;
	cout << "count"; 
	cout << "|bx" ;// << endl;
	cout << "|by" ;//<< P1.y; // << endl;
	//cout << endl;
	cout << "|rx" ;//<< P2.x; //<< endl; 
	cout << "|ry" ;//<< P2.y; //<< endl; 
	//cout << endl;
	cout << "|center (x,y)"; // << center_x << "|" << center_y //<< endl;
	cout << "|rotation (deg)";// << rot << endl;
	//cout << "T_processer=" << T << endl;
	cout << endl;

	cout << T << "|"; //endl;
	cout << ++jcount << "|";// << endl;
	cout << P1.x << "|" ;//endl;
	cout << P1.y << "|" ;//endl;
	cout << P2.x << "|" ;//endl;
	cout << P2.y << "|" ;//endl;
	cout << center_x << "," ;//endl;
	cout << center_y << "|" ;//endl;
	cout << rot << "|" << endl;
	cout << endl;*/

	P1.z=T;P2.z=T;centroid.z = T; //centroid.w = T; //ORIGINAL
	T_lost_centroid.x = T_found;
	T_lost_centroid.y = 0;
	T_lost_centroid.z = 0;
	pub1.publish(P1); 
	//pub2.publish(P2);
	pub3.publish(centroid); 
	pub4.publish(T_lost_centroid);
	loop_rate.sleep();
  }//end while
  return 0;
}//end main


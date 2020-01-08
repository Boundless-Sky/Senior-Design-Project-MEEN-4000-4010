#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("got it!",cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(100);
cerr<<"www"<<endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_recv");
  ros::NodeHandle nh;
  cv::namedWindow("got it!", WINDOW_NORMAL);
  cv::startWindowThread();
  ros::Rate loop_rate(50);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
while (ros::ok()){

ros::spinOnce();
    	loop_rate.sleep();

}
cv::destroyWindow("got it!");
}

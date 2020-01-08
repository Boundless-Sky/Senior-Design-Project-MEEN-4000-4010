/*things to edit:
!!!!!!!!desired location (edit every time the resolution is changed!!!!!!!!
PID gains
*/

//setup_stuff (libraries)
#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>

/* ----- Global Variables ----- */
// this is where robot position should be, center of the image. 
// currently using: 1024 x 576
// if sampling too low.... too big of picture

//robot_x/y/rot are just intialization. 
//4-12 double robot_x = 512; //for 1920 => 960, for 1024 => 512
//4-12 double robot_y = 288; //for 1080 => 540, for 576 => 288
double robot_x = 512;
double robot_y = 288;
double robot_rot = 0;
double dt = .08;
double wx, wy, wz, q0, q1, q2, q3, roll, pitch, yaw;

/* ----- Geometry messages ---- */
geometry_msgs::Vector3 vector;
geometry_msgs::Vector3 pixel_error;
geometry_msgs::Quaternion flight;

//do listen_stuff (subscription to image_total) to get input, dt
//get x, y, rotation, and time 
/*void getCoords(const geometry_msgs::Quaternion& Q){
	robot_x = Q.x;
//	robot_y = Q.y;
	robot_rot = Q.z;
	dt = Q.w;
}
*/

/* ---- Functions ----- */
void getCoords(const geometry_msgs::Vector3& v){
// take code from image_total
	robot_x = v.x;
	robot_y = v.y;
	dt = v.z;
}

void toEulerianAngle(double& q0,double& q1,double& q2,double& q3, double& roll, double& pitch, double& yaw)
{
	/* The point of to euler angle is because flight controller spits sensor data out as quaternion. A3 flight functions
	   take in euler */
	double ysqr = q2 * q2;

	// roll (x-axis rotation)
	double t0 = +2.0 * (q0 * q1 + q2 * q3);
	double t1 = +1.0 - 2.0 * (q1 * q1 + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q0 * q2 - q3 * q1);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q0 * q3 + q1 * q2);
	double t4 = +1.0 - 2.0 * (ysqr + q3 * q3);  
	yaw = std::atan2(t3, t4);
}

void attitude_callback(const dji_sdk::AttitudeQuaternion& attitude_msg_in) {
	//https://developer.dji.com/onboard-sdk/documentation/introduction/things-to-know.html
	//http://download.dji-innovations.com/downloads/dev/OnboardSDK/Onboard_API_introduction_version_1.0.1_en.pdf
	// these link is very useful
	
	//Hamilton quaternion notation
  	q0 = attitude_msg_in.q0;
	q1 = attitude_msg_in.q1;
	q2 = attitude_msg_in.q2;
	q3 = attitude_msg_in.q3;
	//angular velocities (omega = w)
	wx = attitude_msg_in.wx;
	wy = attitude_msg_in.wy;
	wz = attitude_msg_in.wz;

	toEulerianAngle(q0, q1, q2, q3, roll, pitch, yaw);
}

int main(int argc, char **argv){

	//ros setup
	ros::init(argc, argv, "tracker_control");
	ros::NodeHandle nh;

	//setup publisher and subscriber
	//ros::Publisher vec_pub = nh.advertise<geometry_msgs::Vector3>("/cmd_att",1); //for vector3 
	ros::Publisher quat_pub = nh.advertise<geometry_msgs::Quaternion>("/cmd_att",1); //for Quaternion
	ros::Subscriber quat_sub = nh.subscribe("/image_centroid",1,getCoords); //From image_total.cpp
    	ros::Subscriber attitude_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attitude_callback); //Flight Controller Sensors
        ros::Publisher error_pub = nh.advertise<geometry_msgs::Vector3>("/pixel_error",1);

	//4-12 ros::Rate loop_rate(10);
	//set the desired positions and gains
	ros::Rate loop_rate(20);

	/* --- for pitch - about the y axis --> moves in X --- */
	//double des_x = 512;
	double des_x = 200;
	// 4-12 double des_x = 512;// 640 --> 512;		// desired x pixel location
	double des_pitch = 0;		//desired pitch value
	double kp_x = 0.015; 		// proportional gain
	double ki_x = 0;		// integrator term
	double kd_x = 0.012;//.00005;		// derivative term
	double sum_err_x = 0; 		//initialize to 0
	double prev_err_x = 0;  	//initialize to 0

	/* --- for roll - about the x axis --> moves in Y --- make negative! */
	//double des_y = 288;
	double des_y = 200;
	//4-12 double des_y = 288; //480 --> 288; 		// desired y pixel location
	double des_roll = 0;
	double kp_y = -.0035; 		// proportional gain
	double ki_y = 0;		// integrator term
	double kd_y = -0.012; // 0.00005;//-.00005; 		// derivative term
	double sum_err_y = 0;  		//initialize to 0
	double prev_err_y = 0;  	//initialize to 0
	
	// NOTE NOTE NOTE NOTE -> may need a different dt (change in time) since they are tracking against sensors and not camera
	/* --- for thrust ~ use barometer --> z direction is down see documentation --- */
	// GET THIS WORKING SECOND
	double des_t = 0; //stay at 1.2 meters (4 ft) after taking_off command
	//double kp_t = -.015; 		// proportional gain
	//double ki_t = -.006;		// integrator term
	//double kd_t = -.00005; 	// derivative term
	//double sum_err_t = 0;  	//initialize to 0
	//double prev_err_t = 0;  	//initialize to 0

	/* --- for yaw (degree/sec) ~ use flight controller compass --- */
	// GET THIS WORKING FIRST
	double des_yaw = 0;
	double kp_yaw = -.015; 		// proportional gain
	double ki_yaw = -.006;		// integrator term
	double kd_yaw = -.00005; 	// derivative term
	double sum_err_yaw = 0;  	//initialize to 0
	double prev_err_yaw = 0; 	//initialize to 0

	int loop_count = 0;
	printf("PID control is online\r\n");
	

	while(ros::ok())
	{
		// ---> error in position
		double err_x = -robot_x + des_x;
//		double err_x = -pitch + des_pitch;
//		double err_y = -roll + des_roll;
		double err_y = -robot_y + des_y;
		double err_yaw = -yaw + des_yaw; 
	
		// ---> sum of errors (used for integral terms)
		// ---> add next increment like euler method
		sum_err_x += err_x*dt;
		sum_err_y += err_y*dt;
		sum_err_yaw += err_yaw*dt;

		// ---> outputs to be published (PID stuff happens here).
		double flight_roll;
		double flight_pitch;
		double flight_yaw;
		if (loop_count != 0){
			flight_pitch = kp_x*err_x + ki_x*sum_err_x/loop_count + kd_x*(err_x - prev_err_x) / dt;
			flight_roll = kp_y*err_y + ki_y*sum_err_y/loop_count + kd_y*(err_y - prev_err_y) / dt;
			flight_yaw = kp_yaw*err_yaw + ki_yaw*sum_err_yaw/loop_count + kd_yaw*(err_yaw - prev_err_yaw) / dt;
		}

		// --->update errors
		prev_err_x = err_x;
		prev_err_y = err_y;
		prev_err_yaw = err_yaw;
		loop_count++;

		// --->note: in the camera frame, left is increasing x and down is increasing y
		/*
		vector.x = flight_pitch;
		vector.y = 0.0;
		vector.z = 0.0; //set to yaw angle eventually 
		*/
		//if the roll or pitch is above an angle cap, set them to the angle cap
		double angle_max = 10.0;
		if(flight_roll > angle_max){
			flight_roll = angle_max;
		}
		else if(flight_roll < -angle_max){
			flight_roll = -angle_max;
		}
		if(flight_pitch > angle_max){
			flight_pitch = angle_max;
		}
		else if(flight_pitch < -angle_max){
			flight_pitch = -angle_max;
		}
		flight.y = flight_roll;
		flight.x = flight_pitch;
		flight.z = 0;//flight_thrust;
		flight.w = flight_yaw;
		
		pixel_error.x = err_x;
		pixel_error.y = err_y;
		pixel_error.z = 0;
		// --->do publish_stuff to send output
		//vec_pub.publish(vector);
		quat_pub.publish(flight);
		error_pub.publish(pixel_error);

		ros::spinOnce();
		loop_rate.sleep();
	}//end While
	
	return 0;
}// end Main

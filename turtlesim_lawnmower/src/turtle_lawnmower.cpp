/**
 * Program for making turtlesim to move in a lawnmower shape
 * 
 * Author: Venkatarao Rebba <vrebba@asu.edu>
 * Date: Feb 06 2022
 * */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <bits/stdc++.h>
#include <sstream>
#include <cmath>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;
const int speed = 1; // Speed: 1m/sec


void moveTurtle(double speed, double distance, bool isForward);
void rotateTurtle(double angular_speed, double angle, bool isCloclwise);	//this will rotate the turtle at specified angle from its current angle
double conv2Rad(double angle_in_degrees);
void setOrientation(double angle_radians);	//this will rotate the turtle at an absolute angle, whatever its current angle is
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void drawLawnmower();

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_lawnmower");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
	ros::Rate loop_rate(0.5);

	//	/turtle1/cmd_vel is the Topic name
	//	/geometry_msgs::Twist is the msg type
	ROS_INFO("\n\n\n ********Moveing in the direction*********\n");

	drawLawnmower();
	ros::spin();

	return 0;
}

/*Moves the turtlesim with certain speed in the forward/backward direction */
void moveTurtle(double distance, bool isForward)
{
	geometry_msgs::Twist velocityMsg;
	//set a random linear velocity in the x-axis
	if (isForward)
		velocityMsg.linear.x =abs(speed);
	else
		velocityMsg.linear.x =-abs(speed);
		
	velocityMsg.linear.y =0;
	velocityMsg.linear.z =0;
	//set a random angular velocity in the y-axis
	velocityMsg.angular.x = 0;
	velocityMsg.angular.y = 0;
	velocityMsg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double t1;
	double currentDistance = 0.0;
	ros::Rate loop_rate(100);

	while(currentDistance<distance){
		velocity_publisher.publish(velocityMsg);
		t1 = ros::Time::now().toSec();
		currentDistance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}
	velocityMsg.linear.x =0;
	velocity_publisher.publish(velocityMsg);
}

/**
 *  
 *  makes the robot turn with a certain angular velocity in clock/anti-clock wise
 */
void rotateTurtle (double angular_speed, double relative_angle, bool isClockwise)
{
	geometry_msgs::Twist velocityMsg;
	//set a random linear velocity in the x-axis
	velocityMsg.linear.x =0;
	velocityMsg.linear.y =0;
	velocityMsg.linear.z =0;
	//set a random angular velocity in the y-axis
	velocityMsg.angular.x = 0;
	velocityMsg.angular.y = 0;
	if (isClockwise)
		velocityMsg.angular.z =-abs(angular_speed);
	else
	 	velocityMsg.angular.z =abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double currentAngle = 0.0;
	double t1;
	ros::Rate loop_rate(1000);
	while(currentAngle<relative_angle){
		velocity_publisher.publish(velocityMsg);
		t1 = ros::Time::now().toSec();
		currentAngle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	};
	velocityMsg.angular.z =0;
	velocity_publisher.publish(velocityMsg);
}

/*
 Convert angles to radians
*/
double conv2Rad(double angle_in_degrees)
{
	return angle_in_degrees *PI /180.0;
}

/**
 *  turns the robot to a desried absolute angle
 */
void setOrientation(double angleRadians)
{
	double relativeAngle = angleRadians - turtlesim_pose.theta;
	bool clockwise = ((relativeAngle<0)?true:false);
	rotateTurtle (abs(relativeAngle), abs(relativeAngle), clockwise);
}

/**
 * A callback function to update the pose of the robot
 */
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}


/*
 * the cleaning appication function. returns true when completed.
 */
void drawLawnmower()
{
	ros::Rate loop(0.5);
	turtlesim::Pose goal_pose;

    double a = 6.0;	// Fixed width value
    double b = 4.0; // Fixed height value
    double ret = atan(2*b/a);
    double theta = 180 - ((ret * 180) / PI);
    double diagonal = sqrt(4*b*b + (a*a));
    cout <<  " a" <<a << "b" << b << "theta" << theta << "diagonal" << diagonal;

//	loop.sleep();
	setOrientation(0);
	loop.sleep();


	rotateTurtle(conv2Rad(20), conv2Rad(180), true);
	loop.sleep();

	moveTurtle(a/2, true);
	loop.sleep();

	rotateTurtle(conv2Rad(20), conv2Rad(90), true);
	loop.sleep();

	moveTurtle(b,true);
	loop.sleep();

	rotateTurtle(conv2Rad(20), conv2Rad(90), true);
	loop.sleep();

	moveTurtle(a,true);
	loop.sleep();

	rotateTurtle(conv2Rad(20), conv2Rad(theta), true);
	loop.sleep();

	moveTurtle(diagonal, true);
	loop.sleep();

	rotateTurtle(conv2Rad(20), conv2Rad(theta), false);
	loop.sleep();

	moveTurtle(a,true);
	loop.sleep();

	rotateTurtle(conv2Rad(20), conv2Rad(90), false);
	loop.sleep();

	moveTurtle(b,true);
	loop.sleep();

	rotateTurtle(conv2Rad(20), conv2Rad(90), false);
	loop.sleep();

	moveTurtle(a/2,true);
	loop.sleep();

}

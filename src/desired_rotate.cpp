#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

void rotate(double angular_speed, double angle_radian, bool direction);
double deg_to_radian(double angle_deg);
void desired_orientation(double desired_angle, bool direction);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

int main(int argc, char **argv){
	ros::init(argc, argv, "turtlesim_rotate");
	ros::NodeHandle nh;
	double angle_radian;
	double angular_speed;
	double angle_deg;
	double desired_angle_deg;
	bool direction;
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);
	ROS_INFO("----STARTING----");
	cout << "Enter Angle of Orientation in degrees\n";
	cin >> desired_angle_deg;
	//deg_to_radian(angle_deg);
	/*cout << "Enter Angular speed in degrees/sec\n";
	cin >> angular_speed;*/
	//deg_to_radian(angular_speed);
	cout << "Enter 0 for Clockwise, 1 for Anti-Clockwise\n";
	cin >> direction;
	cout << "desired " << turtlesim_pose.theta << endl;
	//angle_radian = deg_to_radian(desired_angle_deg) - turtlesim_pose.theta;
	//rotate(deg_to_radian(50), angle_radian, direction);


	desired_orientation(deg_to_radian(desired_angle_deg), direction);
	ros::Rate rate(10);
	rate.sleep();
	ros::spin();
	return 0;
}


void rotate(double angular_speed, double angle_radian, bool direction){
	geometry_msgs::Twist vel_msg;
	if (direction){
		vel_msg.angular.z = abs(angular_speed);
	}
	else
		vel_msg.angular.z = -abs(angular_speed);
	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(100);
	while (current_angle < angle_radian){
		vel_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		//cout << current_angle*180/3.14 << endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
	cout << "----Reached----\n";
	//cout << "rotate " << turtlesim_pose.theta << endl;
	//cout << "rotate x " << turtlesim_pose.x << endl;
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}

double deg_to_radian(double angle_deg){
	double angle_radian = angle_deg/180*3.142;
	//cout << "degtorad " << turtlesim_pose.theta << endl;
	return angle_radian;
}

void desired_orientation(double desired_angle, bool direction){
	geometry_msgs::Twist vel_msg1;
	vel_msg1.angular.z = 90;
	vel_pub.publish(vel_msg1);
	double angle_radian = desired_angle - turtlesim_pose.theta;
	//cout << desired_angle << endl;
	cout << "desired" << turtlesim_pose.theta << endl;
	//cout << turtlesim_pose.x << endl;
	//cout << angle_radian << endl;
	rotate(deg_to_radian(50), angle_radian, direction);
}
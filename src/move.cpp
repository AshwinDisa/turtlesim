#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

void move(double speed, double distance, bool isForward);

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

int main(int argc, char **argv){
	ros::init(argc, argv, "turtlesim_move_straight");
	ros::NodeHandle nh;
	double speed;
	double distance;
	bool isForward;
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);
	ROS_INFO("---STARTING----\n");
	cout << "Enter Speed\n" << endl;
	cin >> speed;
	cout << "Enter Distance\n" << endl;
	cin >> distance;
	cout << "Enter 1 for Froward, 0 for Reverse\n" << endl;
	cin >> isForward;
	move(speed, distance, isForward);
	ros::Rate loop_rate(10);
	loop_rate.sleep();
	ros::spin();
	return 0;
}

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	if(isForward){
		vel_msg.linear.x = abs(speed);
	}
	else
		vel_msg.linear.x = -abs(speed);
	double current_distance = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate rate(100);
	while (current_distance < distance){
		vel_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);
		ros::spinOnce();
		rate.sleep();
	}
	vel_msg.linear.x = 0.0;
	vel_pub.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}
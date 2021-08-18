#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pos_sub;
turtlesim::Pose turtlesim_pose;

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void spiral(double limit);

int main(int argc, char **argv){
	ros::init(argc, argv, "go_to_goal");
	ros::NodeHandle nh;
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pos_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);
	double limit;
	cout << "----STARTING----\n";
	cout << "Enter Limit less than 9\n";
	cin >> limit;
	spiral(limit);
	ros::Rate rate(10);
	rate.sleep();
	ros::spin();
	return 0;
}

void spiral(double limit){
	geometry_msgs::Twist vel_msg;
	double increment = 1;
	ros::Rate rate(100);
	vel_msg.linear.x = 0.0;
	while (turtlesim_pose.x < limit || turtlesim_pose.y < limit){
		vel_msg.linear.x = vel_msg.linear.x + increment;
		vel_msg.angular.z = 7;
		vel_pub.publish(vel_msg);
		increment=+0.02;
		ros::Rate loop_rate(100);
		ros::spinOnce();
		loop_rate.sleep();
	}
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
	cout << "----FINISHED----" <<endl;
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

void rotate(double angular_speed, double angle_radian, bool direction);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double deg_to_radian(double angle_deg);

int main(int argc, char **argv){
	ros::init(argc, argv, "turtlesim_rotate");
	ros::NodeHandle nh;
	double angle_radian;
	double angular_speed;
	double angle_deg;
	bool direction;
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);
	ROS_INFO("----STARTING----");
	cout << "Enter Angle in degrees\n";
	cin >> angle_deg;
	//deg_to_radian(angle_deg);
	cout << "Enter Angular speed in degrees/sec\n";
	cin >> angular_speed;
	//deg_to_radian(angular_speed);
	cout << "Enter 1 for Clockwise, 0 for Anti-Clockwise\n";
	cin >> direction;
	rotate(deg_to_radian(angular_speed), deg_to_radian(angle_deg), direction);
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
		cout << current_angle*180/3.14 << endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;

}

double deg_to_radian(double angle_deg){
	double angle_radian = angle_deg/180*3.142;
	return angle_radian;
}
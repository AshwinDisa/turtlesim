#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pos_sub;
turtlesim::Pose turtlesim_pose;

void clean_grid();
void rotate(double angular_speed, double angle_radian, bool direction);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double deg_to_radian(double angle_deg);
void move(double speed, double distance, bool isForward);
void go_to_goal(turtlesim::Pose goal_pose, double distance_error);
void desired_orientation(double desired_angle, bool direction);
double get_distance(double x1, double y1, double x2, double y2);



int main(int argc, char **argv){
	ros::init(argc, argv, "go_to_goal");
	ros::NodeHandle nh;
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pos_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

	turtlesim::Pose goal_pose;
	goal_pose.x = 1.0;
	goal_pose.y = 1.0;
	goal_pose.theta = 0;
	go_to_goal(goal_pose, 0.01);

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

void go_to_goal(turtlesim::Pose goal_pose, double distance_error){
	geometry_msgs::Twist vel_msg;
	double Kp = 1.0;
	//double Ki = 0.4;
	ros::Rate loop_rate(100);
	double distance = get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
	while (distance > distance_error){
		double distance = get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.x = Kp * distance;
		vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta); 
		vel_pub.publish(vel_msg);
		//cout << "X = " << turtlesim_pose.x << endl;
		//cout << "Y = " << turtlesim_pose.y << endl;
		ros::spinOnce();
		loop_rate.sleep();
		if (distance < distance_error)
			break;
	}
	//cout << "----Reached----\n";
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
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
	rotate(deg_to_radian(30), angle_radian, direction);
}

void clean_grid(){
	ros::Rate loop(10);
	loop.sleep();
	turtlesim::Pose goal_pose;
	goal_pose.x = 1.0;
	goal_pose.y = 1.0;
	goal_pose.theta = 0.0;
	go_to_goal(goal_pose, 0.01);
	loop.sleep();
	desired_orientation(90, 1);
}

double get_distance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2 - x1),2) + pow((y2 - y1), 2));
}




#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

using namespace std;

ros::Publisher vel_pub;
ros::Subscriber pos_sub;
turtlesim::Pose turtlesim_pose;

void go_to_goal(turtlesim::Pose goal_pose, double distance_error);
double get_distance(double x1, double y1, double x2, double y2);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);


int main(int argc, char **argv){
	ros::init(argc, argv, "go_to_goal");
	ros::NodeHandle nh;
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pos_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

	turtlesim::Pose goal_pose;
	cout << "Enter x co-ordinate\n";
	cin >> goal_pose.x;
	cout << "Enter y co-ordinate\n";
	cin >> goal_pose.y;
	goal_pose.theta = 0;
	go_to_goal(goal_pose, 0.01);
	ros::Rate rate(10);
	rate.sleep();
	ros::spin();
	return 0;
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
		cout << "X = " << turtlesim_pose.x << endl;
		cout << "Y = " << turtlesim_pose.y << endl;
		ros::spinOnce();
		loop_rate.sleep();
		if (distance < distance_error)
			break;
	}
	cout << "----Reached----\n";
	vel_msg.linear.x = 0.0;
	vel_msg.angular.z = 0.0;
	vel_pub.publish(vel_msg);
}

double get_distance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2 - x1),2) + pow((y2 - y1), 2));
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}


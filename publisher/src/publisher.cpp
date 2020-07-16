#include <ros/ros.h>
#include <geometry_msgs/Point.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "grabit");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("grab", 1);
	ros::Rate loop_rate(10);
	geometry_msgs::Point msg;
	msg.x = 0.5;
	msg.y = 0;
	msg.z = 0;
	pub.publish(msg)
	return 0;
}
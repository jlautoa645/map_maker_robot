#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "user_speed_input");
	ros::NodeHandle user_input;

	ros::Publisher pub_user_speed = user_input.advertise<std_msgs::Float64>("user_speed_input", 0);

	ros::Rate loop_rate(1);

	std_msgs::Float64 desired_speed;

	while(ros::ok)
	{
		std::cout << "Enter desired speed in m/s: ";

		std::cin >> desired_speed.data;
		
		pub_user_speed.publish(desired_speed);

		std::cout << std::endl << "Robot is now traveling at: " << desired_speed << " m/s" << std::endl;

		loop_rate.sleep();
	}
	
	return 0;
		
}

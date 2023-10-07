#include <iostream>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"

#define LEFT_WHEEL	0						// assign the wheels to constants for use in 
#define RIGHT_WHEEL	1						// set_pwm function

const float Kp = 4;							// proportional gain constant
const float speed_tolerance = 0.1;					// anywhere within 0.1 m/s is fine

std_msgs::Float64 left_speed_desired, right_speed_desired;		// desired speeds for the left and right motors (m/s)
std_msgs::Float64 left_speed_current, right_speed_current;		// define ros float variables to store the current motor speeds
std_msgs::Int64 left_pwm_out, right_pwm_out;				// define ros int variables to store pwm values to be published to the motor driver

// forward declare callbacks
void setup(void);							// setup function to initialize values
void update_left_speed(std_msgs::Float64 left_speed);			// updates the current speed of the left wheel
void update_right_speed(std_msgs::Float64 right_speed);			// updates the current speed of the right wheel
void update_desired_speed(std_msgs::Float64 desired_speed);		// updates the desired speed
float calculate_error(float current_speed, float desired_speed);	// calculates the error between current and desired values
void set_pwm(int wheel);						// sets the pwm signal given the error between desired and current values

int main(int argc, char **argv)
{
	// initialize ros
	ros::init(argc, argv, "wheel_speed_controller");
	ros::NodeHandle speed_controller;

	// initialize subscribers to motor speed topics
	ros::Subscriber sub_speed_left = speed_controller.subscribe("wheel_speed_left", 0, update_left_speed);
	ros::Subscriber sub_speed_right = speed_controller.subscribe("wheel_speed_right", 0, update_right_speed);
	ros::Subscriber sub_desired_speed = speed_controller.subscribe("user_speed_input", 0, update_desired_speed);

	// initialize publishers to motor driver
	ros::Publisher pub_pwm_left = speed_controller.advertise<std_msgs::Int64>("left_pwm", 0);
	ros::Publisher pub_pwm_right = speed_controller.advertise<std_msgs::Int64>("right_pwm", 0);

	ros::Rate loop_rate(10);	// match the update speed of the publisher

	float left_error, right_error;

	while(ros::ok)
	{
		ros::spinOnce();	// execute callback functions to update speed values

		set_pwm(LEFT_WHEEL);
		set_pwm(RIGHT_WHEEL);
		

		std::cout << "desired speed: " << left_speed_desired.data << std::endl;
		pub_pwm_left.publish(left_pwm_out);
		pub_pwm_right.publish(right_pwm_out);

		loop_rate.sleep();
		
	}

	return 0;
}

void setup(void)
{
	// initialize desired speed values to 0 and pwm output values to 0
	left_speed_desired.data = 0;
	right_speed_desired.data = 0;

	left_pwm_out.data = 0;
	right_pwm_out.data = 0;
}

void update_left_speed(std_msgs::Float64 left_speed)
{
	left_speed_current = left_speed;
}

void update_right_speed(std_msgs::Float64 right_speed)
{
	right_speed_current = right_speed;
}

void update_desired_speed(std_msgs::Float64 desired_speed)
{
	left_speed_desired = desired_speed;
	right_speed_desired = desired_speed;
}

float calculate_error(float current_speed, float desired_speed)
{
	return desired_speed - current_speed;
}

void set_pwm(int wheel)
{
	float error;

	if(wheel == LEFT_WHEEL)
	{
		error = calculate_error(left_speed_current.data, left_speed_desired.data);
		std::cout << "error: " << error << std::endl;
		if(abs(error) > speed_tolerance)
		{
			left_pwm_out.data += Kp * error;
			std::cout << "increment by: " << Kp*error << std::endl;
		}
	}
	else if(wheel == RIGHT_WHEEL)
	{
		error = calculate_error(right_speed_current.data, right_speed_desired.data);
		if(abs(error) > speed_tolerance)
		{
			right_pwm_out.data += Kp * error;
		}
	}
}

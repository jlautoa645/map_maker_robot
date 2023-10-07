#include <iostream>
#include <pigpiod_if2.h>
#include "../include/map_maker_robot/l298n.h"

#include "ros/ros.h"
#include "std_msgs/Int64.h"

int pi;

std_msgs::Int64 left_pwm, right_pwm;

void pi_init(void);						// intialize the pi, set pin directions, initialize pins and constants to certain states
void motors_zero(void);						// function to turn motors off
void motors_forward(void);					// sets the direction of the motors to forwards
void motors_backward(void);					// sets the direction of the motors to backwards
void update_left_pwm(std_msgs::Int64 left_pwm_in);		// callback function to update left wheel pwm signal from controller
void update_right_pwm(std_msgs::Int64 right_pwm_in);		// callback function to update right wheel pwm signal from controller

int main(int argc, char **argv)
{
	pi_init();

	// initialize ros and l298n node
	ros::init(argc, argv, "l298n");
	ros::NodeHandle l298n;

	// initialize subscribers to the controller topics
	ros::Subscriber sub_left_pwm = l298n.subscribe("left_pwm", 0, update_left_pwm);
	ros::Subscriber sub_right_pwm = l298n.subscribe("right_pwm", 0, update_right_pwm);
	
	// loop frequency
	ros::Rate loop_rate(10);

	motors_forward();

	while(ros::ok)
	{
		ros::spinOnce();		// execute callback functions to update pwm values
		
		set_PWM_dutycycle(pi, MOTOR_1_SPEED, left_pwm.data);
		set_PWM_dutycycle(pi, MOTOR_2_SPEED, right_pwm.data);
		
		std::cout << left_pwm.data << std::endl;
	
		loop_rate.sleep();
	}	

	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;
	
	pi = pigpio_start(addr_str, port_str);

	// declare pin directions
	set_mode(pi, MOTOR_1_SPEED, PI_OUTPUT);
	set_mode(pi, MOTOR_1_IN1, PI_OUTPUT);
	set_mode(pi, MOTOR_1_IN2, PI_OUTPUT);
	set_mode(pi, MOTOR_2_SPEED, PI_OUTPUT);
	set_mode(pi, MOTOR_2_IN3, PI_OUTPUT);
	set_mode(pi, MOTOR_2_IN4, PI_OUTPUT);
	
	// set pwm frequency for motors
	set_PWM_frequency(pi, MOTOR_1_SPEED, 5000);
	set_PWM_frequency(pi, MOTOR_2_SPEED, 5000);

	// initialize motors to be off
	motors_zero();
}

void motors_zero(void)
{
	gpio_write(pi, MOTOR_1_IN1, 0);
	gpio_write(pi, MOTOR_1_IN2, 0);

	gpio_write(pi, MOTOR_2_IN3, 0);
	gpio_write(pi, MOTOR_2_IN4, 0);
}

void motors_forward(void)
{
	gpio_write(pi, MOTOR_1_IN1, 1);
	gpio_write(pi, MOTOR_1_IN2, 0);
	
	gpio_write(pi, MOTOR_2_IN3, 1);
	gpio_write(pi, MOTOR_2_IN4, 0);
}

void motors_backward(void)
{
	gpio_write(pi, MOTOR_1_IN1, 0);
	gpio_write(pi, MOTOR_1_IN2, 1);

	gpio_write(pi, MOTOR_2_IN3, 0);
	gpio_write(pi, MOTOR_2_IN4, 1);
}

void update_left_pwm(std_msgs::Int64 left_pwm_in)
{
	left_pwm = left_pwm_in;
}

void update_right_pwm(std_msgs::Int64 right_pwm_in)
{
	right_pwm = right_pwm_in;
}

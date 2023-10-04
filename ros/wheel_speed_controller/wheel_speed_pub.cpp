#include <iostream>
#include <pigpiod_if2.h>
#include <iomanip>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define MOTOR_1_ENC_A	6
#define MOTOR_2_ENC_A 	25

int pi;

int motor_1_A_pulses = 0, motor_2_A_pulses = 0;
const float PULSES_PER_REVOLUTION = 176.3;
const float WHEEL_CIRCUMFERENCE = 0.21;

std_msgs::Float64 motor1_speed, motor2_speed;

void pi_init(void);
void motor_1_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);
void motor_2_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);

int main(int argc, char **argv)
{
	pi_init();

	ros::init(argc, argv, "wheel_speed_publisher");
	ros::NodeHandle wheel_speed;

	ros::Publisher speed_pub_left = wheel_speed.advertise<std_msgs::Float64>("wheel_speed_left", 0);
	ros::Publisher speed_pub_right = wheel_speed.advertise<std_msgs::Float64>("wheel_speed_right", 0);

	ros::Rate loop_rate(10);

	float m1_revs, m2_revs, m1_dist, m2_dist;

	while(ros::ok)
	{
		m1_revs = motor_1_A_pulses / PULSES_PER_REVOLUTION;
		m2_revs = motor_2_A_pulses / PULSES_PER_REVOLUTION;

		m1_dist = m1_revs * WHEEL_CIRCUMFERENCE;
		m2_dist = m2_revs * WHEEL_CIRCUMFERENCE;

		motor1_speed.data = m1_dist*10;
		motor2_speed.data = m2_dist*10;

		speed_pub_left.publish(motor1_speed);
		speed_pub_right.publish(motor2_speed);

		motor_1_A_pulses = 0;
		motor_2_A_pulses = 0;

		loop_rate.sleep();
	}

	pigpio_stop(pi);
	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

	// set pin directions for encoder signals
	set_mode(pi, MOTOR_1_ENC_A, PI_INPUT);
	set_mode(pi, MOTOR_2_ENC_A, PI_INPUT);

	// enable pull up resistors for encoder signals
	set_pull_up_down(pi, MOTOR_1_ENC_A, PI_PUD_UP);
	set_pull_up_down(pi, MOTOR_2_ENC_A, PI_PUD_UP);

	// initialize callback functions
	int motor_1_rising = callback(pi, MOTOR_1_ENC_A, RISING_EDGE, motor_1_enc_A_cb);
	int motor_2_rising = callback(pi, MOTOR_2_ENC_A, RISING_EDGE, motor_2_enc_A_cb);
}

void motor_1_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	motor_1_A_pulses++;		// increment motor 1 channel A pulses on rising edge of signal
}

void motor_2_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	motor_2_A_pulses++;		// increment motor 2 channel A pulses on risinge edge of signal
}

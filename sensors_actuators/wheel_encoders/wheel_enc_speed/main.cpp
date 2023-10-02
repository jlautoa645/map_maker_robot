#include <iostream>
#include <iomanip>
#include <pigpiod_if2.h>
#include "jga25371.hpp"

int pi;
int motor_1_A_pulses = 0, motor_2_A_pulses = 0;

void pi_init(void);
void motor_1_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);
void motor_2_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);

int main()
{
    	pi_init();

	int current_time = get_current_tick(pi);				// get the current time in microseconds

	float m1_revs, m2_revs, m1_dist, m2_dist;
	float m1_speed_ms, m2_speed_ms, m1_speed_fs, m2_speed_fs;

	while(1)
	{
		if((get_current_tick(pi) - current_time) >= 500000)		// update the speed values every half second
		{
			current_time = get_current_tick(pi);

			// calculate motor revolutions
			m1_revs = motor_1_A_pulses / PULSES_PER_REVOLUTION;
			m2_revs = motor_2_A_pulses / PULSES_PER_REVOLUTION;

			// calculate motor distances traveled
			m1_dist = m1_revs * WHEEL_CIRCUMFERENCE;
			m2_dist = m2_revs * WHEEL_CIRCUMFERENCE;

			// calculate motor speed (m/s) by multiplying distance * 2
			// v = d/t where t is 1/2
			m1_speed_ms = m1_dist*2;
			m2_speed_ms = m2_dist*2;

			// convert m/s speed to ft/s speed
			m1_speed_fs = m1_speed_ms*FT_PER_M;
			m2_speed_fs = m2_speed_ms*FT_PER_M;

			// print to user
			std::cout << std::setprecision(3) << "Motor 1: " << m1_speed_ms << " (m/s), " << m1_speed_fs << " (ft/s) " << std::endl;
			std::cout << std::setprecision(3) << "Motor 2: " << m2_speed_ms << " (m/s), " << m2_speed_fs << " (ft/s) " << std::endl;
			std::cout << std::setprecision(3) << "Average: " << (m1_speed_ms+m2_speed_ms)/2 << " (m/s), " << (m1_speed_fs+m2_speed_fs)/2 << " (ft/s) " << std::endl << std::endl;

			// reset pulse counts for next measurement interval
			motor_1_A_pulses = 0;
			motor_2_A_pulses = 0;
		}

	}
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

	//	set all encoder signals as inputs
	set_mode(pi, MOTOR_1_ENC_A, PI_INPUT);
	set_mode(pi, MOTOR_2_ENC_A, PI_INPUT);

	//	enable pull up resistors
	set_pull_up_down(pi, MOTOR_1_ENC_A, PI_PUD_UP);
	set_pull_up_down(pi, MOTOR_2_ENC_A, PI_PUD_UP);

	//	initialize callback functions for the channel A signals of each motor
	int motor_1_rising = callback(pi, MOTOR_1_ENC_A, RISING_EDGE, motor_1_enc_A_cb);
	int motor_2_rising = callback(pi, MOTOR_2_ENC_A, RISING_EDGE, motor_2_enc_A_cb);
}

void motor_1_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	motor_1_A_pulses++;		// increment pulse counter on rising edge
}

void motor_2_enc_A_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	motor_2_A_pulses++;		// increment pulse counter on rising edge
}

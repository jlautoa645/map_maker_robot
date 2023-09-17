#include <iostream>
#include <pigpiod_if2.h>

#define MOTOR_1_SPEED   21
#define MOTOR_1_IN1     26
#define MOTOR_1_IN2     13
#define MOTOR_2_SPEED   19
#define MOTOR_2_IN3     12
#define MOTOR_2_IN4     20

int pi;

void pi_init(void);
void motors_zero(void);
void motors_forward(uint16_t speed);
void motors_backward(uint16_t speed);

int main()
{
	pi_init();

	uint16_t motorspeed, timesec;
	char direction;

	std::cout << "Choose direction. 'f' for forwards, 'b' for backwards." << std::endl;
	std::cin >> direction;

	std::cout << "Choose speed. Enter an integer value betwee 0 and 255." << std::endl;
	std::cin >> motorspeed;

	std::cout << "Choose the amount of time in seconds. Enter an integer value betwen 0 and 255" << std::endl;
	std::cin >> timesec;

	if(direction == 'f')
	{
		std::cout << "Robot is now moving forward at speed " << motorspeed << " for " << timesec << " seconds." << std::endl;
		motors_forward(motorspeed);
		time_sleep(timesec);
        std::cout << "Manuever complete." << std::endl;
	}
	else if(direction == 'b')
	{
		std::cout << "Robot is now moving backward at speed " << motorspeed << " for " << timesec << " seconds." << std::endl;
		motors_backward(motorspeed);
		time_sleep(timesec);
        std::cout << "Manuever complete." << std::endl;
	}
	else
	{
		std::cout << "Invalid input(s). Rerun program and try again." << std::endl;
	}

	motors_zero();

	pigpio_stop(pi);

	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	//	initialize pi
	pi = pigpio_start(addr_str, port_str);

	//	declare pins. in this case theyre all outputs.
	set_mode(pi, MOTOR_1_SPEED, PI_OUTPUT);
	set_mode(pi, MOTOR_1_IN1, PI_OUTPUT);
	set_mode(pi, MOTOR_1_IN2, PI_OUTPUT);
	set_mode(pi, MOTOR_2_SPEED, PI_OUTPUT);
	set_mode(pi, MOTOR_2_IN3, PI_OUTPUT);
	set_mode(pi, MOTOR_2_IN4, PI_OUTPUT);

	//	initialize motors to be off
	motors_zero();
}

void motors_zero(void)
{
    //	for motor 1, setting in1 = in2 = 0 turns motors OFF
	gpio_write(pi, MOTOR_1_IN1, 0);
	gpio_write(pi, MOTOR_1_IN2, 0);

    //	for motor 2, setting in3 = in4 = 0 turns motors OFF
	gpio_write(pi, MOTOR_2_IN3, 0);
	gpio_write(pi, MOTOR_2_IN4, 0);
}

void motors_forward(uint16_t speed)
{
	//	set pwm frequencies for motors
	set_PWM_frequency(pi, MOTOR_1_SPEED, 5000);
	set_PWM_frequency(pi, MOTOR_2_SPEED, 5000);

	//	set dutycycles for motors to control speed
	set_PWM_dutycycle(pi, MOTOR_1_SPEED, speed);
	set_PWM_dutycycle(pi, MOTOR_2_SPEED, speed);

	//	for motor 1, setting in1 = 1 and in2 = 0 rotates motor forwards
	gpio_write(pi, MOTOR_1_IN1, 1);
	gpio_write(pi, MOTOR_1_IN2, 0);

	//	for motor 2, setting in3 = 1 and in4 = 0 rotates motor forwards
	gpio_write(pi, MOTOR_2_IN3, 1);
	gpio_write(pi, MOTOR_2_IN4, 0);
}

void motors_backward(uint16_t speed)
{
	//	set pwm frequencies for motors
	set_PWM_frequency(pi, MOTOR_1_SPEED, 5000);
	set_PWM_frequency(pi, MOTOR_2_SPEED, 5000);

	//	set dutycycles for motors to control speed
	set_PWM_dutycycle(pi, MOTOR_1_SPEED, speed);
	set_PWM_dutycycle(pi, MOTOR_2_SPEED, speed);

	//	for motor 1, setting in1 = 0 and in2 = 1 rotates motor backwards
	gpio_write(pi, MOTOR_1_IN1, 0);
	gpio_write(pi, MOTOR_1_IN2, 1);

	//	for motor 2, setting in3 = 0 and in4 = 1 rotates motor backwards
	gpio_write(pi, MOTOR_2_IN3, 0);
	gpio_write(pi, MOTOR_2_IN4, 1);
}

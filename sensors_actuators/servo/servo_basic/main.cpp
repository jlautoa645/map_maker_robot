#include <iostream>
#include <pigpiod_if2.h>

#define SERVO_PIN	4

int pi;

void pi_init(void);

int main()
{
	pi_init();
	set_PWM_frequency(pi, SERVO_PIN, 50);

	std::cout << "Setting servo to 0 degrees..." << std::endl;
	set_PWM_dutycycle(pi, SERVO_PIN, 6);
	time_sleep(3);

	std::cout << "Setting servo to 90 degrees..." << std::endl;
	set_PWM_dutycycle(pi, SERVO_PIN, 17);
	time_sleep(3);

	std::cout << "Setting servo to 180 degrees..." << std::endl;
	set_PWM_dutycycle(pi, SERVO_PIN, 30);
	time_sleep(3);

	pigpio_stop(pi);

	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

	set_mode(pi, SERVO_PIN, PI_OUTPUT);
	gpio_write(pi, SERVO_PIN, 0);
}

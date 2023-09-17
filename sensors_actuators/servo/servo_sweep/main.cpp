#include <iostream>
#include <pigpiod_if2.h>
#include "servo_data.h"

#define SERVO_PIN	4

int pi;

void pi_init(void);

int main()
{
	pi_init();

	set_PWM_frequency(pi, SERVO_PIN, 50);

	double SERVO_DUTY;
	while(1)
	{
        for(SERVO_DUTY = SERVO_MIN; SERVO_DUTY<SERVO_MAX; SERVO_DUTY+=0.02)
        {
            set_PWM_dutycycle(pi, SERVO_PIN, SERVO_DUTY);
            time_sleep(0.001);
        }
        for(SERVO_DUTY = SERVO_MAX; SERVO_DUTY>SERVO_MIN; SERVO_DUTY-=0.02)
        {
            set_PWM_dutycycle(pi, SERVO_PIN, SERVO_DUTY);
            time_sleep(0.001);
        }
	}

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

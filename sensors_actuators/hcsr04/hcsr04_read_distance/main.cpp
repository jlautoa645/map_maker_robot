#include <iostream>
#include <pigpiod_if2.h>

#define TRIG_PIN    23
#define ECHO_PIN    22

int pi;
int t1, t2;

void pi_init(void);
void echo_rising_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);
void echo_falling_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);

int main()
{
    	int dt;
    	double d_in, d_cm;

	pi_init();

   	for(int i = 3; i>0; i--)
	{
		std::cout << "Sending in: " << i << std::endl;
		time_sleep(1);
	}

	gpio_trigger(pi, TRIG_PIN, 10, 1);
	time_sleep(0.02);
	dt = t2 - t1;

	d_cm = (dt / 2) * 0.0343;
	d_in = d_cm / 2.54;

	std::cout << "Inches: " << d_in << ", Cm: " << d_cm << std::endl;

	while(1){}

	pigpio_stop(pi);

	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

	// set pin directions
	set_mode(pi, TRIG_PIN, PI_OUTPUT);
	set_mode(pi, ECHO_PIN, PI_INPUT);

	// initialize trig pin to output 0
	gpio_write(pi, TRIG_PIN, 0);

	// initialize callbacks
	int echo_rise = callback(pi, ECHO_PIN, RISING_EDGE, echo_rising_cb);
	int echo_fall = callback(pi, ECHO_PIN, FALLING_EDGE, echo_falling_cb);
}

void echo_rising_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	t1 = get_current_tick(pi);
}

void echo_falling_cb(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	t2 = get_current_tick(pi);
}

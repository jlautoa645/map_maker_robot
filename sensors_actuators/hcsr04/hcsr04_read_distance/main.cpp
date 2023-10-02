#include <iostream>
#include <pigpiod_if2.h>

#define TRIG_PIN    23
#define ECHO_PIN    22

int pi;

void pi_init(void);

int main()
{
    	int t1, t2, dt;					// variables will store system ticks (microseconds)
    	float d_in, d_cm;				// variables will store distance measurements

	pi_init();

   	for(int i = 3; i>0; i--)
	{
		std::cout << "Sending in: " << i << std::endl;
		time_sleep(1);
	}

	gpio_trigger(pi, TRIG_PIN, 10, 1);		// provide trigger signal
	
	while(gpio_read(pi, ECHO_PIN) == 0){}		// wait for echo to transition to high
	t1 = get_current_tick(pi);			// get the current time at the transition
	while(gpio_read(pi, ECHO_PIN) == 1){}		// wait for echo to transition to low
	t2 = get_current_tick(pi);			// get the current time at the transition

	dt = t2 - t1;					// calculate the time duration of the echo signal

	d_cm = (dt / 2) * 0.0343;			// calculate distance in centimeters
	d_in = d_cm / 2.54;				// calculate distance in inches

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
}

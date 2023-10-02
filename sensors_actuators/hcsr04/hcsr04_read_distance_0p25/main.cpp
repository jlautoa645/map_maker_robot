#include <iostream>
#include <iomanip>
#include <pigpiod_if2.h>

#define TRIG_PIN	23
#define ECHO_PIN	22

int pi;

void pi_init(void);

int main()
{
	pi_init();

	int t1, t2;
	float d_cm = 0, d_in = 0;

	while(1)
	{
		for(int i = 0; i<5; i++)
		{
			gpio_trigger(pi, TRIG_PIN, 10, 1);		// send trigger signal

			while(gpio_read(pi, ECHO_PIN) == 0){}		// wait for echo to transition high
			t1 = get_current_tick(pi);			// get current cpu time (microseconds)
			while(gpio_read(pi, ECHO_PIN) == 1){}		// wait for echo to transition low
			t2 = get_current_tick(pi);			// get current cpu time (microseconds)

			int dt = t2 - t1;				// calculate the time distance between the rising echo edge and falling echo edge

			d_cm += (dt / 2) * 0.0343;			// calculate distance in cm and add to d_cm
			d_in += ((dt / 2) * 0.0343) / 2.54;				// calculate distance in inches and add to d_in

			time_sleep(0.05);				// 0.25 seconds between each measurement
		}

		// average out the distance measurements
		d_cm = d_cm / 5;
		d_in = d_in / 5;

		// output information to user
		std::cout << std::setprecision(4) << "Distance: " << d_cm << " cm, " << d_in << " in" << std::endl << std::endl;

        // re initialize to 0
        d_cm = 0;
        d_in = 0;
	}

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

	// initialize trig pin to 0
	gpio_write(pi, TRIG_PIN, 0);
}

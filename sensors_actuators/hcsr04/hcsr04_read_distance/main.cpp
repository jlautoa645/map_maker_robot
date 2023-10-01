#include <iostream>
#include <pigpiod_if2.h>

#define TRIG_PIN    23
#define ECHO_PIN    22

int pi;

void pi_init(void);

int main()
{
    int t1, t2, dt;
    double d_in, d_cm;

	pi_init();

    while(1)
    {
        for(int i = 3; i>0; i--)
        {
            std::cout << "Sampling in " << i << " seconds... " << std::endl;
            time_sleep(1);
        }

        gpio_trigger(pi, TRIG_PIN, 10, 1);

        while(gpio_read(pi, ECHO_PIN) == 0){}
        t1 = get_current_tick(pi);
        while(gpio_read(pi, ECHO_PIN) == 1){}
        t2 = get_current_tick(pi);

        dt = t2 - t1;    //microseconds

        d_cm = (dt/2)*0.0343;
        d_in = d_cm / 2.54;

        std::cout << "time: " << dt << " microseconds." << std::endl;
        std::cout << "inches: " << d_in << std::endl;
        std::cout << "centimeters: " << d_cm << std::endl;
    }

	pigpio_stop(pi);

	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

	set_mode(pi, TRIG_PIN, PI_OUTPUT);
	set_mode(pi, ECHO_PIN, PI_INPUT);

	gpio_write(pi, TRIG_PIN, 0);
}

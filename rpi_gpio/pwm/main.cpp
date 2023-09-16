#include <iostream>
#include <pigpiod_if2.h>

#define PWM_GPIO    17

int pi_init(void);

int main()
{

    int pi = pi_init();

    set_PWM_frequency(pi, PWM_GPIO, 5000);

    int i;

    while(1)
    {
        for(i = 1; i<254; i+=2)
        {
            set_PWM_dutycycle(pi, PWM_GPIO, i);
            time_sleep(0.1);
        }
        time_sleep(0.1);
        for(i = 254; i>1; i-=2)
        {
            set_PWM_dutycycle(pi, PWM_GPIO, i);
            time_sleep(0.1);
        }
    }

    pigpio_stop(pi);

    return 0;

}

int pi_init(void)
{
    char *addr_str = NULL;
    char *port_str = NULL;

    int pi = pigpio_start(addr_str, port_str);

    set_mode(pi, PWM_GPIO, PI_OUTPUT);
    gpio_write(pi, PWM_GPIO, 0);

    return pi;
}

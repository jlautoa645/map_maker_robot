#include <iostream>
#include <pigpiod_if2.h>

#define BLINK_PIN   17

int pi_init(void);

int main()
{
    int pi = pi_init();

    //  see if pi connected successfully. if successful pi >= 0
    if(pi>=0)
    {
        std::cout << "Pi connection to daemon successful." << std::endl;
    }
    else
    {
        std::cout << "Pi connection to daemon unsuccessful. Run sudo pigpiod and try again." << std::endl;
    }

    //  infinite blink loop
    while(1)
    {
        gpio_write(pi, BLINK_PIN, 1);
        std::cout << "turning led on..." << std::endl;
        time_sleep(1);
        gpio_write(pi, BLINK_PIN, 0);
        std::cout << "turning led off..." << std::endl;
        time_sleep(1);
    }

    pigpio_stop(pi);

    return 0;

}

int pi_init(void)
{
    //  addr_str specifies the host/IP address running the daemon
    //  addr_str = NULL assigns host = localhost
    char *addr_str = NULL;

    //  port_str specifies the port address used by the pi
    //  port_str = NULL assigns port address to "8888"
    char *port_str = NULL;

    //  connect to pigpio daemon
    int pi = pigpio_start(addr_str, port_str);

    //  configure blink pin as a gpio output
    set_mode(pi, BLINK_PIN, PI_OUTPUT);

    //  initialize blink pin to being off
    gpio_write(pi, BLINK_PIN, 0);

    return pi;
}

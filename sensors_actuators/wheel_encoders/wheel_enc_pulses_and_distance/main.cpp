#include <iostream>
#include <iomanip>
#include <pigpiod_if2.h>
#include "jga25371.hpp"

//	encoder signals. for this sketch we will use the motor 1 encoder signals
#define MOTOR_ENC_A	MOTOR_1_ENC_A
#define MOTOR_ENC_B	MOTOR_1_ENC_B

int pi;
int pulses = 0;

void pi_init(void);
void enc_a_cb_rising(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick);

int main()
{
	int i;
	for(i = 3; i>0; i--)
	{
		std::cout << "Beginning program execution in: " << i << std::endl;
        	time_sleep(1);
	}

	pi_init();

	int current_state_A = gpio_read(pi, MOTOR_ENC_A);           // get the current state of channel A
	int current_time;                                           // declare a variable to store the current time
	int total_pulses = 0;					    // declare a variable to store total amount of pulses
	float revolutions, distance_m, distance_ft;		    // declare variables to store amount of revolutions and distance values

	//	program will be an infinite loop that waits for the encoder to give data
	//	once the encoder begins giving data, the program will begin reading the
	//	amount of pulses from encoder channel A
	while(1)
	{
		while(gpio_read(pi, MOTOR_ENC_A) == current_state_A){}			// while encoder channel A is not changing, do nothing.

		pulses++;								// increment pulses by 1 when A starts changing (first pulse)
		current_time = get_current_tick(pi);					// get the current CPU time in microseconds
		while(pulses != 0)
		{
			if(get_current_tick(pi)-current_time >= 1000000 && pulses>20)	// every second, record the amount of pulses from channel A
			{								// 20 is a noise margin to prevent negligably small movements (pulses<=20) from being registered
				current_time = get_current_tick(pi);
				std::cout << "Pulses: " << pulses << std::endl;

				//	calculate amount of revolutions and distance traveled
				total_pulses += pulses;
				revolutions = total_pulses / PULSES_PER_REVOLUTION;	// calculate the number of revolutions
				distance_m = revolutions * WHEEL_CIRCUMFERENCE;		// calculate the distance in meters
				distance_ft = distance_m * FT_PER_M;			// convert the distance measurement to feet

				std::cout << std::setprecision(3) << "Total distance traveled: " << distance_m << "m, " << distance_ft << "ft" << std::endl;

				pulses = 0;						// set pulses equal to 0 to repeat the loop
			}
		}
	}

	std::cout << "Program execution complete." << std::endl;
	pigpio_stop(pi);
	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

	//	declare encoder phase pins as inputs
	set_mode(pi, MOTOR_ENC_A, PI_INPUT);
	set_mode(pi, MOTOR_ENC_B, PI_INPUT);

	//	enable pull-up resistors to reduce signal noise
	set_pull_up_down(pi, MOTOR_ENC_A, PI_PUD_UP);
	set_pull_up_down(pi, MOTOR_ENC_B, PI_PUD_UP);

	//	initialize callback function for encoder channel a
    	int enc_rising_edge_a = callback(pi, MOTOR_ENC_A, RISING_EDGE, enc_a_cb_rising);
}

void enc_a_cb_rising(int pi, unsigned int user_gpio, unsigned int level, uint32_t tick)
{
	pulses++;				//	increment pulses at the rising edge of the channel A signal
}

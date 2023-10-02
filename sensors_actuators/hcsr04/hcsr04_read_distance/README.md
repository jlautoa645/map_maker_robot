This sketch measures distance once using the hcsr04. It works by sending a trigger signal to the hcsr04, determining the length of the echo pin pulse, and using an equation to calculate the distance.

**Example**

Here are the waveforms for the signals sent to and from the hcsr04

- On the top is the trigger signal. It is a 10 microsecond pulse sent from the rasperry pi to the hcsr04 module. After the hcsr04 receives the signal, it will send a sound wave that will bounce off of the nearest object and reflect back to the sensor.

- On the bottom is the echo signal. After the sensor sends the sound wave, it pulls the echo pin high, and when the wave returns to the sensor it pulls the signal low. We can use the amount of time that the echo pin is high to calculate the distance of the nearest object.

![Screenshot 2023-10-01 203436](https://github.com/jlautoa645/map_maker_robot/assets/121917210/f190d800-f931-46e6-be59-f04911e2f97d)

In this example, the echo pin was high for 2.175 milliseconds, or 2175 microseconds. We can use the equation d=v*t to calculate the distance of the object by substituting 343 m/s as roughly the speed of sound in room temperature air, and t being the amount of time the echo pin was high. We have to divide the time by 2 because the sound wave traveled to the object and back, but we just want one way.

The equation becomes d = (343)(2175*(10^-6)/2) = 0.373 meters.

Converting to centimeters we have 37.3 cm, and to inches we have 14.68 inches.

This is consistent with the terminal output from the sketch which calculated a distance of 37.1 cm, or 14.62 inches.

![terminal_output](https://github.com/jlautoa645/map_maker_robot/assets/121917210/9f93757b-b2d9-4f38-ad26-965343f41fb9)



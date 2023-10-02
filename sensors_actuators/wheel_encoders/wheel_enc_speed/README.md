This sketch outputs the current speed of the robot is m/s and ft/s

**Example**

To demonstrate the functionality of this program, I will use the l298n_move_for_seconds sketch to move the robot an arbitrary distance. After the robot has moved, the wheel_enc_pulses_and_distance sketch will report the distance that the robot has traveled. While the robot is moving, this sketch will monitor the speed of each motor and the average speed of the robot. I will use this average speed and amount of time traveled to calculate the distance traveled by the robot and compare to the distance reported by the wheel_enc_pulses_and_distance sketch.

- l298n_move_for_seconds terminal

![l298n_terminal](https://github.com/jlautoa645/map_maker_robot/assets/121917210/98942474-0edf-4da1-97a7-f3dffb39e646)

In the first part, I use the l298n_move_for_seconds sketch to get the robot to move an arbitrary distance.

- wheel_enc_pulses_and_distance terminal

![wheel_enc_pulses_distance_terminal](https://github.com/jlautoa645/map_maker_robot/assets/121917210/58b3d997-089a-4d36-b3e8-c784d7ac60f9)

The wheel_enc_pulses_and_distance program will report the distance that the robot has traveled. In this example, the robot looks to have traveled 2.59 m, or 8.5 ft.

- wheel_enc_speed terminal

![wheel_enc_speed_terminal](https://github.com/jlautoa645/map_maker_robot/assets/121917210/3660c614-4e03-4bdd-a63e-381478ca2243)


This is the terminal output of the sketch we are testing. It outputs the speed of each motor (motor 1 being the left motor, 2 being the right if looking from the front) and the average speed of both motors every half second.

From the l298n_move_for_seconds program, we told the robot to travel for 3 seconds. To get the average velocity over this time interval, we add up the velocity values from the wheel_enc_speed output and divide by 6 because the program outputs the speed every half second.

This gives us an average velocity of 2.75 ft/s. Multiplying by the amount of time (3 seconds) we get a distance traveled of 8.25 ft, which is about 3% off of our expect value of 8.5 ft.

This sketch reads the encoder signals and determines:

- Pulses read from the wheel encoders each second
- Total distance traveled

**Example**

To demonstrate the functionality of the program, I will place the robot on a measured piece of tape, use the l298n_move_for_seconds sketch to move the robot, and this program will measure the distance traveled by the robot using the wheel encoders.

- Initial setup

![IMG_2522](https://github.com/jlautoa645/map_maker_robot/assets/121917210/be050897-d4ec-4709-9ca2-4d24e7e3683a)



The robot is placed on the measured tape with the front caster wheel positioned at the 0 ft position.

- l298n_move_for_seconds terminal output

![wheel_enc_dist_test_l298n](https://github.com/jlautoa645/map_maker_robot/assets/121917210/fb563253-25e9-4843-92a1-4385da1c707d)


Here, I enter the parameters to get the robot moving with the l298n_move_for_seconds sketch. Although I enter specific direction, speed, and time values for this example, this wheel encoder sketch can calculate distance traveled for any arbitrary values.

- l298n_move_for_seconds robot movement


https://github.com/jlautoa645/map_maker_robot/assets/121917210/0941a460-370b-455d-914e-621e13981088

This is the video of the robot moving according to the parameters specified for the l298n_move_for_seconds program. The numbers on the line are difficult to see in the video, but there is a mark for every 6 inches. The first 6 inches are right in front of the robot.

- wheel_enc_pulses_and_distance terminal output

![wheel_enc_dist_test](https://github.com/jlautoa645/map_maker_robot/assets/121917210/5fc77b9a-9555-4b8a-b475-676e70a41b13)

This is the terminal output from the wheel encoder sketch. The program has counted the total pulses and used this to calculate the traveled distance, which should be about 2.68 feet. Let's see where the robot ended up on the measured tape to see if this is accurate.

- Picture of robot final position

![IMG_2525](https://github.com/jlautoa645/map_maker_robot/assets/121917210/88677815-f2a9-426e-9564-cf89bd3f04b4)

The robot caster wheel which was originally at position 0 ft is now located a little bit past the 2.5 ft mark, which is consistent with out program output of 2.68 ft.

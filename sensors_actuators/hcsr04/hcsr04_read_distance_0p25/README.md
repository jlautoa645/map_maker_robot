This sketch publishes distance data every 0.25 seconds in cm and inches. It works exactly the same as the hcsr04_read_distance sketch but in this sketch I implement a loop to average 5 values before publishing the data. The averaging is aimed to mitigate the effects of randomly high or low values that are read by the sensor, which is a common problem with the module.

- Example terminal output:

![terminal_output2](https://github.com/jlautoa645/map_maker_robot/assets/121917210/9479f5e7-ac6c-418d-8503-08d9c3b53c48)

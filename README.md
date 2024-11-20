## First drive the car outside at the point away from the building, and remain static for
## 30 s
# Then initialize all the sensors
$ cd demo_ws
$ source devel/setup.bash
$ roslaunch basic_launch sensor_init.launch

# Visulaize the GPS
$ source devel/setup.bash
$ roslaunch basic_launch visualization.launch

# Enable joystick control
$ source devel/setup.bash
$ roslaunch basic_launch dbw_joystick.launch

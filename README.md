# hexapod_ros

This packages are for ROS integration for our [hexapods].

## Packages

### hexapod_bringup
launch file for the hardware interface of Pexod

### hexpoad_description
Xacro for Pexod, as well as relevant launch files for the description

### hexapod_driver
C++ API to control our hexapods (talks to dynamixel_control_hw through trajectory messages)

### hexapod_ros
metapackage for the others

## Dependencies

- [dynamixel_control_hw]: Hardware interface for ros_control and the Dynamixels actuators

## Authors

- Author/Maintainer: Konstantinos Chatzilygeroudis
- Other contributors: Dorian Goepp

## LICENSE

[CeCILL]

[hexapods]: http://www.resibots.eu/photos.html#robots
[CeCILL]: http://www.cecill.info/index.en.html
[dynamixel_control_hw]: https://github.com/resibots/dynamixel_control_hw

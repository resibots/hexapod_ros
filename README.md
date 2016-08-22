# hexapod_ros

These packages are for ROS integration for our [hexapods].

## Packages

| Package name        | description                                            |
|---------------------|--------------------------------------------------------|
| hexapod_bringup     | launch file for the hardware interface of Pexod        |
| hexapod_description | Xacro for Pexod, as well as relevant launch files for the description |
| hexapod_driver      | C++ API to control our hexapods (talks to dynamixel_control_hw through trajectory messages) |
| hexapod_ros         | metapackage for the above packages                     |

> **Important:** When this repository is updated, URDF files must be generated and pushed to [hexapod_common]/hexapod_models.

[hexapod_common]:https://github.com/resibots/hexapod_common

## Dependencies

- [dynamixel_control_hw]: Hardware interface for ros_control and the Dynamixels actuators
- [hexapod_common]: Its controllers are used by the `hexapod_driver` package

## Authors

- Author/Maintainer: Konstantinos Chatzilygeroudis
- Other contributors: Dorian Goepp

## LICENSE

[CeCILL]

[hexapods]: http://www.resibots.eu/photos.html#robots
[CeCILL]: http://www.cecill.info/index.en.html
[dynamixel_control_hw]: https://github.com/resibots/dynamixel_control_hw
[hexapod_common]: https://github.com/resibots/hexapod_common

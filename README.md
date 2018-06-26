# hexapod_ros

These packages are for ROS integration for our [hexapods].

## Packages

| Package name        | description                                            |
|---------------------|--------------------------------------------------------|
| hexapod_bringup     | launch file for the hardware interface of Pexod        |
| hexapod_description | Xacro for Pexod, as well as relevant launch files for the description |
| hexapod_driver      | C++ API to control our hexapods (talks to dynamixel_control through trajectory messages) |
| hexapod_ros         | metapackage for the above packages                     |

> **Important:** When this repository is updated, URDF files must be generated and pushed to [hexapod_common]/hexapod_models.

## Dependencies

- [dynamixel_control]: Hardware interface for ros_control and the Dynamixels actuators
- [hexapod_common]: Its controllers are used by the `hexapod_driver` package

## Authors

- Author/Maintainer: Konstantinos Chatzilygeroudis
- Other contributors: Dorian Goepp

## Packages

### `hexapod_description`

Contains the URDF models of our hexapods. Its launch files take care of generating the URDF from Xacro files and loading it in ROS.

The visuals of these models are made with STL files taken from our CAD designs (see [pexod_mechanical_design](https://github.com/resibots/pexod-mechanical-design)) and generated from Robotis' models of their own parts (dynamixel actuators and frames).

## LICENSE

[CeCILL]

[hexapods]: http://www.resibots.eu/photos.html#robots
[CeCILL]: http://www.cecill.info/index.en.html
[dynamixel_control]: https://github.com/resibots/dynamixel_control
[hexapod_common]: https://github.com/resibots/hexapod_common

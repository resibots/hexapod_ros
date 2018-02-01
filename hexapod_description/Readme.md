# Hexapod description

## URDF models of the hexapods

### In ROS

Get the model loaded in `/robot_description` parameter with 
```shell
roslaunch hexapod_description display_hexaforce.launch
```

To also launch an Rviz visualisation of the robot with
```shell
roslaunch hexapod_description display_hexaforce.launch
```

Replace "hexaforce" with "pexod" for the other hexapod.

### Generated URDF files

You can generate URDF models. Just run `catkin_make generate_urdfs`.

`*.urdf` and `*_simulation.urdf` files will be generated in the package source directory. The first ones are to be used with ros packages (since they use the "package://" scheme), while the latters are for other simulators.

> **Note**: The actual difference between these two sets of files is the URI scheme for the STL meshes. For instance, you would find these paths:
>
> - hexaforce.urdf: `package://hexapod_description/stl/chassis.stl`
> - hexaforce_simulation.urdf: `./stl/chassis.stl`
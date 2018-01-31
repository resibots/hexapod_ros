# Hexapod description

## URDF models of the hexapods

## In ROS

Get the model loaded in `/robot_description` parameter with 
```shell
roslaunch hexapod_description display_hexaforce.launch
```

To also launch an Rviz visualisation of the robot with
```shell
roslaunch hexapod_description display_hexaforce.launch
```

Replace "hexaforce" with "pexod" for the other hexapod.

## For simulators and others

You can generate URDF models that do not depend on any ROS utilities. Just run `catkin_make` for this package and the files `pexod_simulation.urdf` and `hexaforce_simulation.urdf` will be generated in the `urdf` folder of this package.

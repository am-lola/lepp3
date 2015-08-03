# lepp3

LOLA Environment Perception Package v3: understanding the environment sensed by
a Kinect-like sensor.

# Goals

The goal of the project is to approximate arbitrary objects in the video feed
of a Kinect-like sensor, using a number of basic geometric shapes, such as
spheres and cylinders, *in real time*.

The approximations can then be used as a basis for a robot control system,
providing it with precise information on the positions of obstacles.

As such, in order to meet the real-time requirement, precision in the
generated approximations is sometimes sacrificed, but with the goal of
never under-approximating an object, so that the robot does not
accidentaly end up colliding with it.

# Background

As part of the research and development of
[humanoid robots](https://www.amm.mw.tum.de/en/research/current-projects/humanoid-robots/)
that the [Institute of Applied Mechanics](https://www.amm.mw.tum.de/en/home/)
of the [Technical University Munich](https://www.tum.de/) does, a
subsystem for obstacle detection based on the sensory information provided
by Kinect-like sensors was developed.

Initially, the system was based on analyzing pure depth maps (2D images,
where each pixel is additionally annotated with depth measurements), as
returned by the sensor. It subsequently moved to leveraging
[PCL](http://pointclouds.org/), in the hopes of simplifying, as well as
improving the subsystem by making use of the algorithms already implemented
by PCL.

This project represents the latest evolution of this subsystem. It builds
upon ideas that were previously implemented in the vision subsystem,
improving the object approximation algorithms, implementing some new approaches
for obtaining the best possible approximations, as well as optimizing and improving
the efficiency and modularity of the previously existing codebase.

It also extracts the purely vision-related components of the subsystem --
the obstacle detection and approximation -- into an **independent** and
reusable C++ library. The library is found in the
[`src/lepp3`](https://github.com/am-lola/lepp3/tree/master/src/lepp3)
directory.

This not only facilitates better code organization and modularity
of the humanoid robot system, but also contributes back to the open source
community, by openly sharing the exact implementation of the object/obstacle
detection and approximation used by the Institute in its humanoid robot
systems.

The implementation of the vision subsystem is also found within this
repository, in the
[`src/lola`](https://github.com/am-lola/lepp3/tree/master/src/lola) tree.
Most of the code there represents communication logic between the robot
control and the vision subsystems, such as obtaining the robot's kinematic
parameters and sending the detected obstacles' descriptions back to the control,
according to previously specified communication protocols. This can be
considered as a good example how the `lepp3` library can be used within the
context of a larger system.

Whether the vision subsystem executable gets built is controlled by the
`LEPP_BUILD_LOLA` option passed to `cmake` when generating the build
configuration (on by default).

The `detector` executable, on the other hand, built when
`LEPP_BUILD_DETECTOR` is set (on by default), is independent from any robot
specifics and displays the detected obstacles' approximations by overlaying
them over a point cloud displayed in PCL's `PCLVisualizer`.


# Compiling

The project depends on the [PCL](http://pointclouds.org/) library. You should
first install it following PCL's directions.

Then, you can compile the `detector` using `cmake`. In order to compile the
project with only the default options set, use the `build.sh` script.

# Usage

The `detector` executable is built by default and has a number of flags
that can be passed to it from the CLI. Check its usage by running it with
no flags.

For some examples of how to use the library itself, you may check the
`examples` directory.

# License

The project is published under the terms of the
[MIT License](https://github.com/am-lola/lepp3/blob/master/LICENSE).

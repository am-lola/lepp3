# lepp3

LOLA Environment Perception Package v3: understanding the environment sensed by
a Kinect-like sensor.
Previous version: [lepp2](https://github.com/am-lola/lepp2)

# Goals

The goal of the project is to approximate arbitrary objects in the video feed
of a 3D sensor, using a number of basic geometric shapes, such as polygons,
spheres and cylinders, *in real time*.

The approximations can then be used as a basis for a robot control system,
providing it with precise information on the positions of surfaces and obstacles.

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

It is divided into different sections, where the
[`src/lepp3`](https://github.com/am-lola/lepp3/tree/master/src/lepp3)
directory contains the purely vision-related components, which are sepparated 
from the humanoid communication files and external dependencies. This 
facilitates better code organization and modularity of the humanoid robot system.

The implementation of the robot communication subsystem is also found within this
repository, in the
[`src/lola`](https://github.com/am-lola/lepp3/tree/master/src/lola) tree.
Most of the code there represents communication logic between the robot
control and the vision subsystems, such as obtaining the robot's kinematic
parameters and sending the detected surfaces and obstacles' descriptions back to 
the control, according to previously specified communication protocols. This can 
be considered as a good example how the `lepp3` library can be used within the
context of a larger system.

# Dependencies

* [PCL](http://pointclouds.org/) (compiled from source with C++11 support)
  * [Instructions to build PCL with C++11 and OpenNI Support]()
* [kalman](https://github.com/mherb/kalman) (a compatible version is provided under [src/deps](./src/deps))
* [ARVisualizer](https://github.com/am-lola/ARVisualizer)

# Compiling

Once the necessary dependencies have been obtained and installed, building is just:

```bash
mkdir build && cd build
cmake ..
make
```

# Usage

The `lola` executable requires only one argument, a config file to load. The config
file contains settings for the specific components to be enabled and their parameters.
The [config](./config) directory contains several configuration files to get started.

```bash
./lola <config_file_path>
```

By adding or removing components in the config file, the executable can be made to
run anything from a simple view from a connected camera, playback or recording of
data obtained from a 3D sensor, or even the complete system involving communication
with a robot and/or other networked components (e.g. to perform AR visualization on a [Hololens](https://github.com/am-lola/HoLola)).

See [master-cfg.toml](./master-cfg.toml) for an example of every available component
and all the possible parameters which can be set. Note, however, that some components
are incompatible, so the master-cfg file cannot be used as-is.

To aid with testing and development, the [`src/lola/iface`](https://github.com/am-lola/lepp3/tree/master/src/lola/iface) folder includes several
tools to send and receive data for the various components involved when using a
real robot (e.g. artifical kinematic data can be sent to lepp3 and lepp3's results
can be broadcast to a mock receiver).


# Structure

The `lepp3` library is inherently modular and flexible. To get started in its 
different components, it helps to look at the structure used for environment 
modeling during [experiments](./config/lab-lola.toml) with the robot LOLA (the 
different components can be activated/deactivated at will through the config file):

![Alt text](https://g.gravizo.com/svg?
  digraph G {
    aize ="4,4";
    main [label="Video Source / Pose Data",shape=box];
    input [label="Input Filter",shape=box];
    detector [label="SurfaceDetector",shape=box];
    visualizer [label="Visualizer / Aggregator",shape=box];
    plane [label="SurfaceFinder",shape=parallelogram];
    inlier [label="Inlier Finder",shape=ellipse];
    gmm [label="GMM Segmenation",shape=ellipse];
    euclidean [label="Euclidean Segmentation",shape=ellipse];
    object [label="Object Approximator",shape=ellipse];
    lowpass [label="Euclidean Low-Pass-Filter",shape=ellipse];
    kalman [label="Euclidean Kalman-Filter",shape=ellipse];
    surfcluster [label="SurfaceClusterer",shape=hexagon];
    surftracker [label="SurfaceTracker",shape=hexagon];
    convexhull [label="ConvexHullDetector",shape=hexagon];    
    main -> input [weight=8];
    input -> detector [weight=8];
    detector -> plane [weight=8,color=".7 .3 1.0",label="Plane Detection"];
    plane -> detector [weight=8,color=".7 .3 1.0"];
    detector -> surfcluster [weight=8,color=".3 .7 1.0",label="Surface Approximation"];
    surfcluster -> surftracker [weight=8,color=".3 .7 1.0"];
    surftracker -> convexhull [weight=8,color=".3 .7 1.0"];
    convexhull -> detector [weight=8,color=".3 .7 1.0"];
    detector -> inlier [weight=8,label="Obstacle Approximation"];
    inlier -> gmm [weight=8];
    inlier -> euclidean [weight=8];
    gmm -> object [weight=8];
    euclidean -> object [weight=8];
    object -> lowpass [weight=8,style=dotted];
    object -> kalman [weight=8,style=dotted];
    object -> visualizer [weight=8];
    lowpass -> visualizer [weight=8,style=dotted];
    kalman -> visualizer [weight=8,style=dotted];
  }
  )

# Benchmarks

The program contains a few tracepoints for tracelogging. It uses the [LTTng](http://lttng.org/)
tool ([version 2.9](http://lttng.org/docs/v2.9/#doc-ubuntu) or higher). In order 
to activate tracelogging, you need to enable the corresponding flag when running 
cmake:

```bash
cmake -DLEPP_ENABLE_TRACING=TRUE ..
```

Then, you need to run LTTng in another terminal before running lepp:

```bash
lttng create
lttng enable-events -u -a
lttng start
<Run lepp>
lttnh stop
lttng destroy
```

A small script is available under `src/script/trace-eval.py` to extract some useful metrics from the resulting trace data (although you can use any trace viewer capable of reading CTF trace data as well). The script requires Python 3, babeltrace (`apt-get install babeltrace libbabeltrace-ctf-dev python3-babeltrace`) and the Python Plot.ly lib (`pip3 install plotly`).

Usage:

```bash
python3 src/script/trace-eval.py <trace_directory> <output_name>

    <trace_directory> : Directory containing the actual trace data
    <output_name>     : Name to give the .html page containing the graph plots

 e.g.
python3 src/sript/trace-eval.py ~/lttng-traces/auto-20171010-236640/ust/uid/1000/64-bit/ plot_output
```

The script will print some statistics about each event found in the trace, and create an .html page `<output_name>.html` containing a plot of each event across the duration of the trace (in frames).

# Publications

The ideas behind this code are published under:

[Wahrmann, Daniel; Hildebrandt, Arne-Christoph; Bates, Tamas; Wittmann, Robert; Sygulla, Felix; Seiwald, Philipp; Rixen, Daniel: Vision-Based 3D Modeling of Unknown Dynamic Environments for Real-Time Humanoid Navigation. International Journal of Humanoid Robotics Vol. 16 No. 01, 2019](http://mediatum.ub.tum.de/node?id=1435457)
[Wahrmann, Daniel; Hildebrandt, Arne-Christoph; Wittmann, Robert; Sygulla, Felix; Rixen, Daniel; Buschmann, Thomas: Fast Object Approximation for Real-Time 3D Obstacle Avoidance with Biped Robots. IEEE International Conference on Advanced Intelligent Mechatronics, 2016](http://mediatum.ub.tum.de/node?id=1320267)

# Videos

The results of these algorithms can be seen on:

[Compilation of Autonomous Walking](https://youtu.be/EeDR1UNDpIY)
[Vision System of Humanoid Robot LOLA](https://youtu.be/VceqNJucPiw)
[Live Demo and Workshop](https://youtu.be/g6UACMHgt20)
[Walking over a platform](https://youtu.be/rKsx8HKvBkg)
[Fast Object Detection and Approximation](https://youtu.be/6PLN6B4vSHM)

# License

The project is published under the terms of the
[MIT License](https://github.com/am-lola/lepp3/blob/master/LICENSE).

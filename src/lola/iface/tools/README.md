# Tools to aid development & testing of networked robot components

Collected here are a few tools for sending and receiving data between different networked components related to the Lola project.

### footstep_msg_server

A server which will broadcast a sequence of planned footsteps using the same structure used by the step planner.
It will send a sequence of 8 planned steps in a loop.

### pose_msg_client

A UDP client for receiving pose data.

### pose_msg_server

A UDP client which will broadcast artificial pose data. The pose data is set to match the lab's camera rig (an asus xtion 1.8m heigh pointing downwards at around 50 degrees).

### vision_msg_client

A client which will receive data sent by lepp3.

### vision_msg_server

A server which will send obstacle and surface data in place of lepp3.

# Build Instructions

The tools above can all be compiled for Linux, QNX, and Windows.

To build all of the tools for your current platform:

```bash
mkdir build && cd build
cmake ..
make
```

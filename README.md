# MuJoCo_StateUpdater

A simple ROS package for listening to various sensor callbacks (joint states, cameras
etc...) and using this information to visualise a real world scene in MuJoCo.

The package is primarily designed to be used to publish a world scene using a simple
custom ros message. There

## Requirements
- ROS Noetic
- MuJoCo

## Custom message type
This package implements a geenric scene message that can be used to isntntiate your physics 
simulator. This message is called "Scene".

Scene.msg
 - Robots[]
 - Objects[]

Robot.msg
- string name
- float64[] joint_positions
- float64[] joint_velocities

Object.msg
- string name
- float64[] pose (x, y, z, qx, qy, qz, w)
- float64[] velocity (x, y, z, wx, wy, wz)
- float32 confidence

## Running

Build packages with `catkin_make` first.

### Scene publisher

``` rosrun MuJoCo_StateUpdater MuJoCo_StateUpdater```

### Scene publisher and visualiser

``` roslaunch MuJoCo_StateUpdater demo_vis.launch```

## TODO
- [ ] Make the package more programmatic, arguments for what robots / objects to track and what sensors to use.
- [ ] Add support for more sensors (realSense for example)
- [ ] Beautify the README

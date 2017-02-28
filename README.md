# Building Exploration

## Exploration Algorithms

During the course of this project, we developed two methods of interpreting movements for the Jackal which were both based on the SICK LIDAR lm2xx input. These algorithms are centered around the idea of following a wall around the building.

### Discrete Wall Avoidence - by [Akhil Kurup](https://github.com/amkurup)
Source: [jackal_move.py](catkin_ws/src/building_mapper/scripts/jackal_move.py)

### Continuous Wall Avoidence - by [Ian Wakely](https://github.com/raveious)
Source: [wall_avoid.py](catkin_ws/src/building_mapper/scripts/wall_avoid.py)

Uses quadratic functions to evaluate the importance of a particular range value read from the LIDAR and evaluates every points along the scan to come to a desired action of avoidence.

# Development

## Completed Jackal connection Tutorial
- [x] [Ian Wakely](https://github.com/raveious) (Project Leader)
- [x] [Phillip Scramlin](https://github.com/pdscraml)
- [x] [Akhil Kurup](https://github.com/amkurup)

## Local Development

After cloning this repository, follow these steps to setup the development environment
```
$ cd catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## Viewing the robot model

Make sure to source all of setup scripts, this will setup all the paths for your local environment.

```
$ source catkin_ws/devel/setup.bash
$ source catkin_ws/remote-jackal.sh
```

View the model via roslaunch and Rviz

```
$ roslaunch building_mapper view_model.launch
```

## Connecting to the Jackal

Once the Jackal is powered up and online, setup the ROS master target to the Jackal

```
$ source catkin_ws/remote-jackal.sh jackal2
```

This will setup everything up to have all your nodes use Jackal 2 as your master node.

## Launch files

Name|Description
:---|:---:
[exploration](catkin_ws/src/building_mapper/launch/exploration.launch)|Main experiment of exploration. This launches everything that the Jackal needs in order to run our exploration routines like the SICK LIDAR and controllers, in addition to our exploration nodes. This is intended to be launched on the Jackal.
[jackal](catkin_ws/src/building_mapper/launch/jackal.launch)|Self contained launch file that will launch all the sensors, their drivers or supporting nodes. This doesn't include any nodes that will cause the robot to move, simply creates a platform to conduct higher order functions. This is intended to be launched on the Jackal.
[sicklms](catkin_ws/src/building_mapper/launch/sicklms.launch)|Configures and launches all nodes that are necessary for the SICK lm2xx LIDAR, the LIDAR we're using for out experiments. This is intended to be launched on the Jackal.
[teleop](catkin_ws/src/building_mapper/launch/teleop.launch)|Testing launch file that records all the data while allowing for teleoperation. This is intended to be launched on the Jackal.
[view_model](catkin_ws/src/building_mapper/launch/view_model.launch)|Allows for the viewing of the Jackal model with all of the additional sensors. No actual control is available.This is intended to be launched on your local machine.
[view_remote_robot](catkin_ws/src/building_mapper/launch/view_remote_robot.launch)|Allows for the viewing of the Jackal model and visualizes all of the additional sensor data. This is intended to be launched on your local machine **and the controllers should already be launched on the Jackal.**
[view_robot](catkin_ws/src/building_mapper/launch/view_robot.launch)|Allows for the viewing of the Jackal model and visualizes all of the additional sensor data. This is intended to be launched on your local machine **and the controllers will be launched on your local machine.**

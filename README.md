# Building Exploration

- Teams should develop a URDF model for the LMS200/291. You may assume the mass of the LIDAR to be uniform.
- Add the URDF model to the Jackal Description.
- Define an appropriate TF in which to publish 'laser scan' data from the SICK.
- Use the SICK Toolbox wrapper to publish LIDAR data in this reference frame.
- Demonstrate operation of the Jackal and LIDAR using rviz.
- You should assume you will have no Wi-Fi connectivity. Start the Jackal moving randomly or following some pattern for a predefined time.
- Document your approach, use of the package, and describe your strategy on the github wiki page.
- Present a map of a floor on the EERC. Include this map on your github wiki page.
- Demonstrate operation of your mapping program at a TBD location after presentations on March 1.

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

### Viewing the robot model

Make sure to source all of setup scripts, this will setup all the paths for your local environment.

```
$ source catkin_ws/devel/setup.bash
$ source catkin_ws/remote-jackal.sh
```

View the model via roslaunch and Rviz

```
$ roslaunch building_mapper view_model.launch
```

## Exploration with the Jackal

Once the Jackal is powered up and online, ssh into it and launch the exploration nodes.

```
$ ssh jackal2
$ cd building-exploration/catkin_ws
$ git pull
$ source devel/setup.bash
$ source remote-jackal.sh
$ roslaunch building_mapper exploration.launch
```

On your local machine, set the ROS master target to the Jackal.

```
$ source catkin_ws/remote-jackal.sh jackal2
```

This will setup everything up to have all your nodes use Jackal 2 as your master node. To view the visualized data from the sensors, use roslaunch on your local machine.

```
$ source catkin_ws/devel/setup.bash
$ roslaunch building_mapper view_remote_robot.launch
```

## Exploration with the Jackal without Wireless Access

Once the Jackal is powered up and online, ssh into it and launch the joystick controller sequence.

```
$ ssh jackal2
$ cd building-exploration/catkin_ws
$ git pull
$ source devel/setup.bash
$ source remote-jackal.sh
$ roslaunch building_mapper joy_start.launch
```

After the Jackal has been started, feel free to drive it to the testing area. Once there, select one of the key combinations on the controller to issue commands.

Key Combination|Sent Command
:---:|:---:
L2 + R2 + Square|Select discrete algorithm ([Known Issues](https://github.com/raveious/building-exploration/wiki#known-issues))
L2 + R2 + Circle|Select continuous algorithm ([Known Issues](https://github.com/raveious/building-exploration/wiki#known-issues-1))
L2 + R2 + Triangle|Stop exploration
L2 + R2 + X|Start exploration

## Handling ROSBAGs

After running the launch files, a rosbag.bag file would be created in the rosbag directory. Make sure to copy it to the local machine (using scp or some other method) to avoid overwriting of the bag file. While launching the replay.launch file to map the bag, make sure to have the correct bag file in the rosbag directory with the name rosbag.bag.

*Note: If the bag has a .active extention, it implies that the bag was left active when the node/robot was shutdown. At this point a reindexing will be needed to recover the bag.
```
$ rosbag reindex rosbag.bag
```

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
[replay](catkin_ws/src/building_mapper/launch/replay.launch)|To be used for map creation. Launches gmapping with SLAM mapping and other configuration settings, followed by rviz. A python script is then launched for the playing back of rosbag data and map generation with map_saver. Intended to be run on local machine. Rosbag data must be copied from jackal after running exploration launch file.
[joy_start](catkin_ws/src/building_mapper/launch/joy_start.launch)|This is intended for the cases where the Jackal can not be accessed wirelessly. This allows for you to launch the Jackal and then drive it to where you want to conduct testing. The tests are then started by using keys on the controller.

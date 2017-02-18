# Building Exploration

## Completed Jackal connection Tutorial
- [x] [Ian Wakely](https://github.com/raveious) (leader)
- [ ] [Phillip Scramlin](https://github.com/pdscraml)
- [ ] [Akhil Kurup](https://github.com/amkurup)

## Development

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

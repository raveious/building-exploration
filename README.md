# Building Exploration

## Completed Jackal connection Tutorial
- [ ] [Ian Wakely](https://github.com/raveious) (leader)
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

## Connecting to the Jackal

Once the Jackal is powered up and online, setup the ROS master target to the Jackal

```
$ source catkin_ws/remote-jackal.sh jackal2
```

This will setup everything up to have all your nodes use Jackal 2 as your master node.

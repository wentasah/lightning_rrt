# lightning_rrt
ROS 2 Jazzy package for obtaining 2D navigation paths using the Rapidly-exploring Random Tree (RRT) algorithm. Custom messages for this package can be found in [lightning_rrt_interfaces](https://github.com/david-dorf/lightning_rrt_interfaces)

## Installation
1. Install ROS 2 Jazzy
2. Install this package and interfaces using the following command:
```bash
sudo apt install ros-jazzy-lightning-rrt ros-jazzy-lightning-rrt-interfaces
```

## Usage
See [example_map.cpp](https://github.com/david-dorf/lightning_rrt/blob/main/src/example_map.cpp) for sending an RRT request. Running the example is as follows:

Terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run lightning_rrt example_map
```

Terminal 2:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run lightning_rrt visualize
```

Terminal 3:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2
```

Path, map, and start/goal markers can be visualized and saved to a config through the rviz2 UI.

## Roadmap
Adding an example rviz2 config, launch file, and tests. RRT variants would be cool.

# lightning_rrt
ROS 2 Jazzy package for obtaining 2D navigation paths using the Rapidly-exploring Random Tree (RRT) algorithm. Custom messages for this package can be found in [lightning_rrt_interfaces](https://github.com/david-dorf/lightning_rrt_interfaces)


https://github.com/user-attachments/assets/d12ce5e2-bf8f-4276-b6c6-0f8e2d99c03d


## Installation
1. Install ROS 2 Jazzy
2. Install this package and its interfaces:
```bash
sudo apt install ros-jazzy-lightning-rrt ros-jazzy-lightning-rrt-interfaces
```

## Usage
See src/example_map.cpp for publishing an RRT request. Run the example with:
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch lightning_rrt launch.xml
```

## Nodes
Run with `source /opt/ros/jazzy/setup.bash | ros2 run lightning_rrt <node_name>`
- example_map: Publishes an example RRT request with a map, start, and goal
- lightning_rrt: Subscribes to rrt_request, publishes path if RRT succeeds
- visualize_rrt: Subscribes to rrt_request, publishes map and start/goal markers
for visualization
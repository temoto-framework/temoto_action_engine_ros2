# temoto_action_engine_ros2
ROS2 wrapper for TeMoto Action Engine

## Installation Instructions
```bash
# Get the dependencies
sudo apt install libboost-all-dev
```

```bash
# Go to the source folder your ROS2 workspace
cd <PATH-TO-YOUR-ROS2-WS>/src

# Download this repository
git clone --recursive https://github.com/temoto-framework/temoto_action_engine_ros2

# Build it
cd <PATH-TO-YOUR-ROS2-WS>
source /opt/ros/<YOUR-ROS2-DISTRO>/setup.bash
colcon build
```

### In Progress
Run the examples

```bash
# Run the Action Engine
ros2 run temoto_action_engine_ros2 action_engine_node \
--actor-name David \
--actions-path src/temoto_action_engine_ros2/examples

# Invoke a UMRF graph
ros2 topic pub /umrf_graph_start --once temoto_msgs/msg/UmrfGraphStart "umrf_graph_name: 'example_sequential_nav'  
name_match_required: false  
targets: [David]
umrf_graph_json: ''  
umrf_graph_diffs: []"
```

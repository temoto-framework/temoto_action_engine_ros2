# temoto_action_engine_ros2
ROS2 wrapper for TeMoto Action Engine

## Installation Instructions
```bash
# Get the dependencies
sudo apt install libclass-loader-dev
```

```bash
# Go to the source folder your ROS2 workspace
cd <PATH-TO-YOUR-ROS2-WS>/src

# Download this repository
git clone --recursive https://github.com/temoto-framework/temoto_action_engine_ros2

# Build the core librart of the action engine
# NB: DON'T SOURCE YOUR ROS2 ENVIRONMENT YET
cd temoto_action_engine_ros2/temoto_action_engine
mkdir build && cd build && cmake .. && make
sudo make install

# Build it
cd <PATH-TO-YOUR-ROS2-WS>
source /opt/ros/<YOUR-ROS2-DISTRO>/setup.bash
colcon build
```

### In Progress
Run the Action Engine

```bash
# Set the libraries path, so that the libtemoto_action_engine.so is picked up from /usr/local/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Run the Action Engine
ros2 run temoto_action_engine_ros2 action_engine_node
```

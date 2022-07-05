# ODRI ROS2 Package for Interface

Collection of ROS2 nodes to interface with the ODRI master-board

> In construction :construction_worker:

## Installations

### Master-Board Sdk

```shell
git clone git@github.com:open-dynamic-robot-initiative/master-board.git
cd master-board/sdk/master_board_sdk
git submodule init
mkdir build && cd build
cmake ..
make
sudo make install
```

More information about the library [here](https://github.com/open-dynamic-robot-initiative/master-board).

### ODRI Control Interface

```shell
git clone git@github.com:open-dynamic-robot-initiative/odri_control_interface.git
cd odri_control_interface
git submodule init
mkdir build && cd build
cmake ..
make
sudo make install
```

More information about the library [here](https://github.com/open-dynamic-robot-initiative/odri_control_interface).

## Useful Command Lines

:warning: Need to launch the nodes in **sudo mode**

```shell
sudo su
ros2 launch odri_ros2_interface _robot_state_machine.launch.py
```

```shell
# To start calibration
ros2 service call /robot_state_machine/state_transition odri_ros2_msgs/srv/TransitionCommand "{command: 'calibrate'}"
# To finish calibration
ros2 service call /robot_state_machine/state_transition odri_ros2_msgs/srv/TransitionCommand "{command: 'enable'}"
# To start running
ros2 service call /robot_state_machine/state_transition odri_ros2_msgs/srv/TransitionCommand "{command: 'start'}"
# To stop running
ros2 service call /robot_state_machine/state_transition odri_ros2_msgs/srv/TransitionCommand "{command: 'stop'}"
# To disable
ros2 service call /robot_state_machine/state_transition odri_ros2_msgs/srv/TransitionCommand "{command: 'disable'}"
```

## Troubleshooting

#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
export LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH
source /home/unitree/capo_ws/install/setup.bash
source /home/unitree/unitree_api_ws/install/setup.bash
source /home/unitree/go2-nav2-amcl/install/setup.bash

exec ros2 launch go2_navigation2 go2_nav2.launch.py "$@"

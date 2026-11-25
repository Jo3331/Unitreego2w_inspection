#!/usr/bin/env bash
# fusion_estimator 端侧环境 (Ubuntu 20.04 / ROS 2 Foxy / Orin NX)
#
# 用法： source ~/capo_ws/env.sh
#
# 三个必须注意的点：
#   1. 必须 source ~/cyclonedds_ws —— /opt/ros/foxy 自带 CycloneDDS 0.7.0 太旧，
#      不认 <Interfaces> 语法，会报 "Interfaces: unknown element"。
#   2. CYCLONEDDS_URI 用机器人自带的 cyclonedds.xml，它已绑定 eth10 (192.168.123.18)。
#   3. RMW 必须显式指定 cyclonedds。

source /opt/ros/foxy/setup.bash
source "$HOME/cyclonedds_ws/install/setup.bash"
source "$HOME/capo_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$HOME/cyclonedds_ws/cyclonedds.xml"

echo "[env.sh] Foxy + cyclonedds_ws + capo_ws 已 source；CYCLONEDDS_URI -> eth10"

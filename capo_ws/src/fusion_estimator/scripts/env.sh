#!/usr/bin/env bash
#
# 一次 source，配好跑 fusion_estimator 全链路所需的环境。
#
# 用法：
#   source scripts/env.sh
#   ros2 run fusion_estimator unitree_lowstate_adapter_node.py
#
# 关键点：~/.bashrc 里的 CYCLONEDDS_URI 指向 ~/.cyclonedds/cyclonedds.xml，
# 那个文件只绑定了 lo（回环）网卡，导致节点收不到机器狗 eth0 上的数据。
# 这里覆盖为绑定 eth0，才能和 192.168.123.18 通信。
#
# 注意：必须用 source 执行，不要用 ./env.sh（那样 export 不会留在当前 shell）。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/hjj/unitree_ros2/install/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

echo "[env.sh] ROS2 humble + unitree_ros2 + fusion_estimator 已 source"
echo "[env.sh] CYCLONEDDS_URI -> eth0"

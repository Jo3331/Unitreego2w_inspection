# Mid-360 3D Navigation Test Workspace

This workspace is prepared for the following ROS 2 Humble chain:

```text
Livox MID-360 driver -> Elevator-LIO -> jie_3d_nav / OctoMap -> SCAN-Planner ROS 2
```

## Source layout

- `src/livox_ros_driver2`: symlink to `/home/hjj/livox_ws/src/livox_ros_driver2`.
- `src/elevator_lio`: `xiaofan4122/Elevator-LIO`.
- `src/jie_3d_nav`: `6-robot/jie_3d_nav`.
- `src/scan_planner_ros2`: `wuyi2121/SCAN-Planner`, branch `ros2-community`.
- `vendor/scan_planner_fork_ros1`: the originally requested fork, kept for reference only.

The existing `/home/hjj/point_lio_ros2` and Livox workspaces were not modified.

## Build status

Successfully built:

- Existing `livox_ros_driver2` install reused from `/home/hjj/livox_ws/install`.
- `lio` (Elevator-LIO).
- `jie_map_msgs` and the SCAN-Planner ROS 2 core packages.
- `scan_planner` and `scan_planner_msgs`.
- `plan_env`, `path_searching`, `bspline_opt`, and `traj_utils`.
- `jie_octomap` and `octo_planner` after installing Open3D.
- `local_sensing_node` after installing GLM; this is optional and only supports a virtual/rendered sensor test.

The first full rebuild after renaming the workspace was run with one parallel worker to avoid memory pressure. Existing source trees were not changed by this rebuild.

## Current ROS interfaces

Elevator-LIO's default Livox input is configured in `src/elevator_lio/yaml/sensors/livox.yaml`:

- LiDAR: `/livox/lidar`
- IMU: `/livox/imu`

The ROS 2 SCAN-Planner launch file exposes `is_real_world`, `navi_mode`, `sensor_type`, `controller_mode`, `use_gpu`, and PCD map options. For global-path tracking use `navi_mode:=3`; the ROS 2 port consumes an external `initial_path` and publishes the optimized trajectory on its planning topics.

`jie_3d_nav` is intended to publish a `nav_msgs/Path` global path, normally `/planned_path`. An adapter/remap is still needed to connect that path to SCAN-Planner's `initial_path` interface and to align the pose/cloud topic names.

## Next commands

The required system packages are already installed:

```bash
sudo apt install libopen3d-dev libglm-dev
cd /home/hjj/Go2w_3D_Nav
source /opt/ros/humble/setup.bash
source /home/hjj/livox_ws/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The simulator package is not required for the real Mid-360 chain. It is only useful when generating synthetic point clouds before the robot and sensor are available.

Before launching real data, verify the Livox topics and configure the MID-360 network/driver settings. Then launch Elevator-LIO with `start_ros2.launch.py`, verify `/LIO/odom_vehicle`, `/LIO/odom_imu`, and the registered cloud output, load an OctoMap in `jie_3d_nav`, and finally run SCAN-Planner with `navi_mode:=3` after the path adapter is in place. The complete hardware runtime chain has not yet been tested because no Mid-360 data stream is currently connected.

# Unitree 端侧 Foxy 部署说明

本工作空间是 `/home/hjj/Go2w_3D_Nav`（Humble 版）的 **Foxy 专用副本**，部署到 Unitree
机器人 `192.168.123.18`。本机 Humble 工作空间未被改动。

状态：**25 个包全部编译通过**，四条导航链路（LIO / JIE / PCT→SCAN / DDDMR）加离线
PCD 流程均已在机器人上启动验证。

---

## 一、目标环境

| 项目 | 本机（Humble） | 机器人（Foxy） |
|---|---|---|
| 主机 | — | `192.168.123.18`，用户 `unitree`，主机名 `ubuntu` |
| 架构 | x86_64 | **aarch64（Jetson, tegra）** |
| 系统 | Ubuntu 22.04 | **Ubuntu 20.04.5** |
| ROS | Humble | **Foxy** |
| Python | 3.10 | **3.8.10** |
| PCL | 1.12 | **1.10** |
| GCC / CMake | — | 9.4.0 / 系统 3.16.3（依赖编译用 3.27.9） |
| CUDA | — | 11.4 |

工作空间位置：`/home/unitree/Go2w_3D_Nav`

---

## 二、⚠️ 机器人 ROS 环境：必须用 CycloneDDS

**这是最容易踩的坑。** 机器人的 `~/.bashrc` 里 ROS 环境分两段：

```bash
# 交互式菜单（选 1 才设 RMW）
echo "ros:foxy(1) noetic(2) ?"; read choose
case $choose in
1) source /opt/ros/foxy/setup.bash
   source ~/cyclonedds_ws/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
   ... ;;
esac

# 无条件执行的部分（只有 Foxy + livox）
source /opt/ros/foxy/setup.bash
source /home/unitree/livox_ws/install/setup.bash
```

也就是说：**非交互 shell（脚本、`bash -c`）拿不到 RMW 设置，会退回默认的 FastRTPS。**

在 FastRTPS 下 `lio` 会刷屏报错并卡死：

```text
Multi thread started
bad_alloc caught: std::bad_alloc   ← 该字符串来自 libfastrtps.so，不是本工作空间的代码
```

`lio` 的 IMU 订阅队列深度是 100000（`imu_cbk`），FastRTPS 会按深度预分配内存而失败。
换用 CycloneDDS 后一切正常。**所以每个终端都要按下面这样初始化：**

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
source ~/livox_ws/install/setup.bash
source ~/Go2w_3D_Nav/install/setup.bash
```

（在机器人桌面开终端时选菜单的 `1` 等价于前四行。）

---

## 三、依赖准备

### 3.1 apt 依赖

```bash
sudo apt-get install -y \
  ros-foxy-octomap ros-foxy-octomap-msgs ros-foxy-sensor-msgs-py \
  ros-foxy-pybind11-vendor ros-foxy-tf2-sensor-msgs ros-foxy-angles \
  ros-foxy-laser-geometry ros-foxy-image-geometry ros-foxy-pcl-ros \
  libglm-dev nlohmann-json3-dev python3-pip pybind11-dev \
  libeigen3-dev libyaml-cpp-dev libpcl-dev libopencv-dev \
  libtbb-dev libarmadillo-dev libtinyxml2-dev libboost-all-dev
```

> `ros-foxy-pcl-ros` 是 Elevator-LIO 的 `find_package(pcl_ros)` 必需的，出厂镜像没有预装。

### 3.2 源码编译的依赖

Ubuntu 20.04 / Foxy 没有 GTSAM、OSQP 1.0 的软件包，需源码编译，统一装在
`/home/unitree/go2_deps/install`：

```bash
# CMake：系统 3.16.3 太旧（OSQP 1.0 要求 >= 3.18）。
# 必须用 3.27.x；CMake 4.x 会因 GTSAM 4.2 声明 cmake_minimum_required(VERSION 3.0) 而报错。
python3 -m pip install --user "cmake==3.27.9"

export PATH="$HOME/.local/bin:$PATH"   # 仅编译依赖时需要

# OSQP 1.0.0
cd ~/go2_deps/src/osqp-1.0.0 && mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/go2_deps/install/osqp-1.0.0 \
      -DCMAKE_BUILD_TYPE=Release .. && make -j4 && make install

# GTSAM 4.2
cd ~/go2_deps/src/gtsam-4.2 && mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/go2_deps/install/gtsam-4.2 \
      -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
      -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON .. \
  && make -j4 && make install
```

> `small_gicp` 已**不再需要**：DDDMR 原来只用它做体素降采样，已改用 `pcl::VoxelGrid`
> （见第五节）。机器人上可以不用编译它。

依赖路径不需要手工导出环境变量——各包 `CMakeLists.txt` 已内建 `GO2_DEPS_ROOT`
查找逻辑，且 RPATH 已开启。

### 3.3 Open3D

C++ 侧不需要（`pcd_to_octomap_node` 已改用 PCL 读 PCD，Open3D 查看器改为可选编译）。

**Python 侧需要**（`pcd_map_import_gui`、`pcd_file_publisher`、`open3d_octomap_viewer`、
`pcd_map_publisher`）：

```bash
pip3 install --user open3d==0.16.0 --no-deps
pip3 install --user plotly dash nbformat configargparse
```

必须 `--no-deps`：Open3D 声明 `numpy>=1.18`，而 ROS Foxy 依赖系统的 numpy 1.17.4。
实测 `--no-deps` 下 Open3D 0.16.0 的 `io`/`geometry`/`KDTreeFlann` 在 numpy 1.17.4 上正常，
且不会动到 ROS 的 numpy。后四个是 `visualization` 模块的纯 Python 依赖，不牵连 numpy。

---

## 四、编译与运行

### 4.1 编译

```bash
cd /home/unitree/Go2w_3D_Nav
source /opt/ros/foxy/setup.bash
source $HOME/livox_ws/install/setup.bash

colcon build --symlink-install --parallel-workers 3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Jetson 只有 4 核、15 GB 内存，`--parallel-workers` 建议不超过 3。

> **必须 source `livox_ws`**，否则 `find_package(livox_ros_driver2)` 失败，`lio` 编译不过。

### 4.2 运行

依赖路径已写进各包 `CMakeLists.txt` 并启用 RPATH，**不需要任何 shell 环境脚本**，
`source install/setup.bash` 之后直接 `ros2 launch`。每个终端按第二节初始化即可。

#### 建图定位：Livox 驱动 + Elevator-LIO

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
ros2 launch lio start_ros2.launch.py                    # 带 RViz
ros2 launch lio start_ros2.launch.py use_rviz:=false    # 无 RViz
```

话题：`/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar`、`/LIO/global_map`。

#### 链路 A：JIE 3D Nav（PCD → OctoMap → 全局路径）

```bash
# 导入 PCD（GUI 选文件）
ros2 launch jie_octomap import_pcd_map.launch.py

# 加载已保存的地图包 + Web 规划界面
ros2 launch jie_octomap web_octomap.launch.py \
  map_package:=$HOME/Go2w_3D_Nav/src/elevator_lio/Octomap/mapv6_level \
  launch_map_gui:=false
# 浏览器访问 http://127.0.0.1:8080
```

#### 链路 B：PCT → SCAN（离线 PCD，已验证）

```bash
# 1) 生成 tomogram（无 CuPy 时自动走 CPU 路径）
ros2 launch tomography tomography.launch.py \
  rsg_root:=$HOME/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2 scene_name:=go2

# 2) 只跑 PCT
ros2 launch pct_planner planner.launch.py \
  scene_name:=go2 \
  tomogram_path:=$HOME/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/mapv6_level.pickle \
  frame_id:=map optimize_path:=false launch_rviz:=true

# 3) PCT + SCAN 联合
ros2 launch pct_scan_bridge paper_scene_pct_scan.launch.py \
  scene_name:=go2 \
  tomogram_path:=$HOME/Go2w_3D_Nav/src/elevator_lio/PCD/pct_go2/tomogram/mapv6_level.pickle \
  pcd_map_file:=$HOME/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd \
  use_rviz:=true use_gpu:=false enable_local_sensing:=false \
  controller_mode:=open_loop plan_on_startup:=false
```

#### 单独跑 SCAN（离线 PCD）

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=3 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:=$HOME/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_level.pcd
```

接实机时（确认坐标系与安全后再用）：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true navi_mode:=3 sensor_type:=lidar controller_mode:=closed_loop
```

#### 链路 C：DDDMR（离线 PCD → 全局路径）

```bash
ros2 launch dddmr_pcd_publisher dddmr_pcd_global_planner.launch.py \
  pcd_file:=$HOME/Go2w_3D_Nav/src/elevator_lio/PCD/mapv6_floor_level.pcd \
  frame_id:=map robot_x:=0.0 robot_y:=0.0 robot_z:=0.0
```

---

## 五、Foxy 适配改动清单

### 5.1 C++

| 问题 | 原因 | 处理 |
|---|---|---|
| `node->now()` / `this->now()` / 裸 `now()` | `rclcpp::Node::now()` 是 Galactic 才加的 | 改为 `get_clock()->now()`。**注意 `clock_->now()` 是合法的，不能改**（`rclcpp::Clock` 本来就有 `now()`） |
| `#include <X.hpp>`：`tf2_geometry_msgs` / `tf2/utils` / `tf2_eigen` / `cv_bridge` / `image_geometry` | Foxy 只有 `.h` | 用 `__has_include` 守卫（`tf2_geometry_msgs`、`tf2_eigen` 的 `toMsg`/`fromMsg` 在 Foxy 里存在，只是文件名不同） |
| 订阅回调写成 `const MsgPtr&` | Foxy 的 `AnySubscriptionCallback` 只接受**按值**传 shared_ptr，Humble 放宽了 | 改为按值。涉及 `grid_map`、`scan_replan_fsm`、`lio`（`LIONode.h`/`callbacks.cpp`/`main.cpp`/`service.cpp`）。**`message_filters` 的回调必须保留 `const&`** |
| lambda 里用 `const XConstPtr&` | 同上 | 改为按值 |
| `message_filters::Subscriber::subscribe()` | Humble 才有的接口 | 改用 `(node, topic, qos)` 构造函数 |
| `pcl::index_t` | PCL 1.11+ 才有 | 改为 `int` |
| `PCL::common` / `PCL::io` / `PCL::filters` imported target | PCL 1.11+ 才有 | 改为 `${PCL_LIBRARIES}`（`jie_octomap`） |
| 链接报 `cannot find -lvtkXXX` | `PCLConfig.cmake:732` 无条件把 `${VTK_LIBRARIES}` 追加进 `PCL_LIBRARIES`；arm64 的 VTK 只有版本化 soname（`libvtkX-7.1.so`），没有 `libvtkX.so` | 5 个用到 `${PCL_LIBRARIES}` 的包加 `list(FILTER PCL_LIBRARIES EXCLUDE REGEX "vtk")` |
| `FLANN::FLANN` 被 re-export | PCL 1.10 把 FLANN 作为 imported target 放进 `PCL_LIBRARIES`；`plan_env` 导出链接接口，下游没有该 target | `list(REMOVE_ITEM PCL_LIBRARIES FLANN::FLANN)`（与 `elevator_lio` 去 `QHULL::QHULL` 同一手法） |
| `rosidl_get_typesupport_target()` | Humble 专有 CMake 命令 | 改为 Foxy 的 `rosidl_target_interfaces()`（`dddmr_sys_core`、`lio`） |
| `tf2_ros::CreateTimerROS` 三参数构造 | Humble 多了 callback group 参数 | 改为 Foxy 的两参数 `(node_base, node_timers)` |
| `declare_parameter(name, PARAMETER_TYPE)` | 「只声明类型」是 Galactic/Humble 重载 | 改为 `declare_parameter(name)`（值仍来自参数文件） |
| `rclcpp::SignalHandlerOptions` | 该枚举与 `init()` 的 signal 参数是 Humble 才有的 | `ros_compat.h` 改用 Foxy 的 `init(argc, argv, InitOptions())` |
| `rcl_yaml_node_struct_init` / `rcl_parse_yaml_file` / `rclcpp::ParameterMap` 未声明 | 符号在 Foxy 里**存在**，但头文件不再被 `rclcpp.hpp` 间接引入 | 显式加 `#include <rcl_yaml_param_parser/parser.h>` 和 `#include <rclcpp/parameter_map.hpp>` |
| `CMAKE_BUILD_RPATH "/usr/lib/x86_64-linux-gnu"` | x86_64 硬编码 | 改为 `CMAKE_LIBRARY_ARCHITECTURE` 推导 |
| Open3D（`jie_octomap`） | 20.04 无 Open3D 开发包 | PCD 读取改 PCL；查看器改可选编译 |
| `small_gicp`（`static_assert: Old PCL version`） | 要求 PCL ≥ 1.11，且返回 `std::shared_ptr` 与 PCL 1.10 的 boost 版 `PointCloud::Ptr` 不兼容 | 3 处调用只做体素降采样，`pcl::VoxelGrid` 同样是逐体素取质心，语义等价。全部替换并移除 `small_gicp` 依赖 |

### 5.2 Python（3.10 → 3.8）

| 文件 | 问题 | 处理 |
|---|---|---|
| `pct_planner/plan.py` | `match`/`case` 是 3.10 语法 | 改为 `if`/`elif` |
| `octo_planner/launch/nav.launch.py`、`web_test.launch.py` | 运行时求值的 `tuple[str, str, str]` | 加 `from __future__ import annotations` |

全树已用机器人上的 Python 3.8 通过 `compileall` 语法检查。

### 5.3 构建与依赖

- PCT 的 GTSAM/OSQP：`CMakeLists.txt` 里通过 `GO2_DEPS_ROOT`（默认
  `$HOME/go2_deps/install`）查找，配合 RPATH，不需要环境脚本。
- `pct_planner/lib/CMakeLists.txt` 加了 `-Wl,--disable-new-dtags`：
  `libgtsam.so` 自身依赖同目录的 `libmetis-gtsam.so` 却没有 RPATH，而 DT_RUNPATH
  **不传递给间接依赖**；改成 DT_RPATH 后整条链可解析，无需 `LD_LIBRARY_PATH`。
- 原 `tools/source_*.sh` 已删除（用户要求不提供 sh 执行脚本）。

### 5.4 运行期行为差异

| 问题 | 原因 | 处理 |
|---|---|---|
| `pcd_file:=...` 覆盖无效 | **Foxy 中参数文件压过命令行 `-p`**（Humble 相反） | 从 `dddmr_static_map.yaml` 移除 `pcd_file`/`frame_id`，改由 launch 参数提供 |
| `static_transform_publisher --x ...` 报参数数量错误 | Foxy 只接受位置参数 `x y z yaw pitch roll frame_id child_frame_id` | 改为位置参数 |
| RViz 报 `rviz_common/Time` 面板加载失败 | 该面板 Foxy 没有 | 从 14 个 `.rviz` 配置中移除该面板条目 |

---

## 六、验证结果

在机器人上以 CycloneDDS 环境实测（无报错，节点持续运行）：

| 链路 | 命令 | 结果 |
|---|---|---|
| Livox + Elevator-LIO | `ros2 launch lio start_ros2.launch.py` | 配置加载正常，节点运行 |
| JIE 3D Nav | `web_octomap.launch.py map_package:=.../Octomap/mapv6_level` | 5 个节点启动，`jie_path_node` 就绪 |
| DDDMR 离线 PCD | `dddmr_pcd_global_planner.launch.py pcd_file:=...` | 发布 `mapcloud=10095 / mapground=644`（输入 131895），`global_planner` 完成地面图构建 |
| SCAN（离线 PCD） | `run.launch.py is_real_world:=false use_pcd_map:=true` | 7 个节点启动，`map_pub` 加载 316493 点，FSM 进入 `INIT` |
| PCT tomography | `tomography.launch.py rsg_root:=... scene_name:=go2` | 生成 `8f_to_9f.pickle`（8.7 MB），CPU 路径正常 |
| PCT planner | `planner.launch.py tomogram_path:=...` | 加载 tomogram，发布 117830 个可通行单元 |
| PCT → SCAN 联合 | `paper_scene_pct_scan.launch.py ...` | 8 个节点启动，bridge 发布 155427 个单元，交互式规划就绪 |

---

## 七、已知限制

- **地图数据未同步**：`src/elevator_lio/PCD/` 在本机有 6.7 GB 建图数据，按约定只传了代码，
  该目录在机器人上为空。上面引用 `PCD/` 的命令都需要先把对应的 PCD / tomogram 传过去。
  仓库自带的小地图（`scan_planner_ros2/map.pcd`、`dddmr_*/data/*.pcd`）和 JIE 的
  Octomap 包（`src/elevator_lio/Octomap/`）已随代码同步。
- **未接真实机器狗**：`scan_planner` 输出到 Unitree 底层控制的适配仍未完成，实机请先保持
  `controller_mode:=open_loop` / `is_real_world:=false`。
- **CuPy / GPU tomography**：机器人上未装 CuPy，`tomography` 走 CPU 路径（已验证可用）。
  需要 GPU 加速时要另装适配 JetPack CUDA 11.4 的 CuPy。
- **`odin_ros_driver` 缺失**：`octo_planner` 的 `exec_depend` 引用了 Odin1 激光雷达驱动，
  机器人上没有该包。只影响 `odin1_loc.launch.py` / `odin1_slam.launch.py` 与
  `nav_params*.yaml` 里的重定位地图路径，不影响 Mid-360 链路。
- **`lio` 的 IMU 队列深度 100000** 在 FastRTPS 下会 `bad_alloc`。本工作空间按机器人约定
  使用 CycloneDDS，不受影响；若将来要回到 FastRTPS，需要先调小该深度。

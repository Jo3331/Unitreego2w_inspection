#!/usr/bin/env bash
set -e

source /opt/ros/foxy/setup.bash
source "$HOME/Go2w_3D_Nav/install/setup.bash"

# The original C++ open3d_loc is linked against the ARM64 prebuilt SDK.
export Open3D_DIR="$HOME/.local/open3d141/lib/cmake/Open3D"
export CMAKE_PREFIX_PATH="$HOME/.local/open3d141:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$HOME/.local/open3d141/lib:$LD_LIBRARY_PATH"

# The workspace was historically built with isolated installs; expose the
# newly added Python package explicitly to the ROS 2 package index.
export AMENT_PREFIX_PATH="$HOME/Go2w_3D_Nav/install/open3d_loc_py:$AMENT_PREFIX_PATH"
export AMENT_PREFIX_PATH="$HOME/Go2w_3D_Nav/install/open3d_loc_native:$AMENT_PREFIX_PATH"
export PYTHONPATH="$HOME/Go2w_3D_Nav/build/open3d_loc_py:$HOME/Go2w_3D_Nav/install/open3d_loc_py/lib/python3.8/site-packages:$PYTHONPATH"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth10</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
fi

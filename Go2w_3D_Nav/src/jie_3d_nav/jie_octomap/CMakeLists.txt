cmake_minimum_required(VERSION 3.8)
project(jie_octomap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(jie_map_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(visualization_msgs REQUIRED)
find_library(TINYXML2_LIB tinyxml2 REQUIRED)
find_package(std_msgs REQUIRED)

# The PCD reader uses PCL. Open3D is only needed by the optional desktop viewer,
# and Ubuntu 20.04 / Foxy has no Open3D development package for the Jetson, so
# the viewer is off unless explicitly requested.
find_package(PCL REQUIRED COMPONENTS common io filters)

# PCL 1.10 appends VTK's library names to PCL_LIBRARIES even when only
# common/io/filters are requested. On arm64 the VTK packages ship versioned
# sonames only (libvtkX-7.1.so, no libvtkX.so), so a bare -lvtkX entry makes the
# link fail; nothing in this package uses VTK. FLANN::FLANN is dropped for the
# same reason: it is an imported target that consumers of this package's
# exported interface would not have defined.
list(FILTER PCL_LIBRARIES EXCLUDE REGEX "vtk")
list(REMOVE_ITEM PCL_LIBRARIES FLANN::FLANN)
find_package(Eigen3 REQUIRED)
find_package(Open3D QUIET)
if(Open3D_FOUND)
  set(JIE_WITH_OPEN3D_DEFAULT ON)
else()
  set(JIE_WITH_OPEN3D_DEFAULT OFF)
endif()
option(JIE_WITH_OPEN3D "Build the Open3D-based octomap viewer" ${JIE_WITH_OPEN3D_DEFAULT})
if(NOT Open3D_FOUND AND JIE_WITH_OPEN3D)
  message(FATAL_ERROR "JIE_WITH_OPEN3D=ON but Open3D was not found")
endif()

ament_python_install_package(${PROJECT_NAME})

add_executable(octomap_test src/octomap_test.cpp)
ament_target_dependencies(octomap_test
  rclcpp
  octomap_msgs
  visualization_msgs
)
target_link_libraries(octomap_test octomap)

add_executable(octomap_to_cloud_node src/octomap_to_cloud_node.cpp)
ament_target_dependencies(octomap_to_cloud_node
  rclcpp
  octomap_msgs
  sensor_msgs
)
target_link_libraries(octomap_to_cloud_node octomap)

add_executable(octomap_to_occupied_markers_node src/octomap_to_occupied_markers_node.cpp)
ament_target_dependencies(octomap_to_occupied_markers_node
  rclcpp
  octomap_msgs
  visualization_msgs
)
target_link_libraries(octomap_to_occupied_markers_node octomap)

add_executable(occupancy_grid_to_octomap_node src/occupancy_grid_to_octomap_node.cpp)
ament_target_dependencies(occupancy_grid_to_octomap_node
  rclcpp
  nav_msgs
  octomap_msgs
)
target_link_libraries(occupancy_grid_to_octomap_node octomap)

if(JIE_WITH_OPEN3D)
  add_executable(octomap_open3d_viewer_node src/octomap_open3d_viewer_node.cpp)
  ament_target_dependencies(octomap_open3d_viewer_node
    rclcpp
    nav_msgs
    octomap_msgs
    visualization_msgs
  )
  target_link_libraries(octomap_open3d_viewer_node
    octomap
    Open3D::Open3D
  )
endif()

add_executable(rviz_click_selector_node src/rviz_click_selector_node.cpp)
ament_target_dependencies(rviz_click_selector_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

add_executable(world_to_octomap_node src/world_to_octomap_node.cpp)
ament_target_dependencies(world_to_octomap_node
  rclcpp
  octomap_msgs
  std_msgs
  visualization_msgs
)
target_include_directories(world_to_octomap_node PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(world_to_octomap_node
  octomap
  ${TINYXML2_LIB}
)

add_executable(pcd_to_octomap_node src/pcd_to_octomap_node.cpp)
ament_target_dependencies(pcd_to_octomap_node
  rclcpp
  octomap_msgs
  std_msgs
)
# PCL 1.10 (Foxy) has no PCL::common / PCL::io / PCL::filters imported targets;
# only the variables are populated. ${PCL_LIBRARIES} works on 1.10 and 1.12.
target_include_directories(pcd_to_octomap_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_to_octomap_node
  octomap
  ${PCL_LIBRARIES}
)

set(JIE_OCTOMAP_TARGETS
  octomap_test
  octomap_to_cloud_node
  octomap_to_occupied_markers_node
  occupancy_grid_to_octomap_node
  rviz_click_selector_node
  world_to_octomap_node
  pcd_to_octomap_node
)
if(JIE_WITH_OPEN3D)
  list(APPEND JIE_OCTOMAP_TARGETS octomap_open3d_viewer_node)
endif()

install(TARGETS ${JIE_OCTOMAP_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/open3d_octomap_viewer.py
  scripts/pcd_file_publisher.py
  scripts/no_cache_http_server.py
  scripts/save_bin_map_gui.py
  scripts/world_file_picker_gui.py
  scripts/world_selector_gui.py
  scripts/web_click_selector.py
  scripts/map_package_manager
  scripts/map_save_gui
  scripts/map_viewer_gui
  scripts/ros_map_import_gui
  scripts/pcd_map_import_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config web worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

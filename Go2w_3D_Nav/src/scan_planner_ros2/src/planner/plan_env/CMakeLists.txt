cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
# PCL 1.10 appends VTK's library names to PCL_LIBRARIES even when only
# common/io/filters are requested. On arm64 the VTK packages ship versioned
# sonames only (libvtkX-7.1.so, no libvtkX.so), so a bare -lvtkX entry makes the
# link fail; nothing here uses VTK. FLANN::FLANN is dropped for the same reason
# at the interface level: plan_env re-exports its link interface and downstream
# packages never define that imported target.
list(FILTER PCL_LIBRARIES EXCLUDE REGEX "vtk")
list(REMOVE_ITEM PCL_LIBRARIES FLANN::FLANN)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(plan_env src/grid_map.cpp src/raycast.cpp)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env
  cv_bridge geometry_msgs message_filters nav_msgs pcl_conversions rclcpp
  sensor_msgs tf2 tf2_ros visualization_msgs
)
target_link_libraries(plan_env Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS plan_env EXPORT export_plan_env
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge Eigen3 geometry_msgs message_filters nav_msgs pcl_conversions
  rclcpp sensor_msgs tf2 tf2_ros visualization_msgs)
ament_package()

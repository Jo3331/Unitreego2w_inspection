cmake_minimum_required(VERSION 3.16)
project(pct_planner)

# Set this before ament generates its install rules. The vendored library
# CMakeLists previously set it too late, which broke symlink installation.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# This Jetson also ships a python3.9 that has no development headers, and
# CMake's FindPython3 prefers the highest version. ROS Foxy runs on Python 3.8,
# so select that interpreter explicitly. Python3_EXECUTABLE can still be
# overridden from the command line or the environment.
if(NOT DEFINED Python3_EXECUTABLE)
  find_program(Python3_EXECUTABLE NAMES python3.8 python3)
endif()
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "pct_planner using Python3: ${Python3_EXECUTABLE} (${Python3_VERSION})")

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('platlib', vars={'platbase': '.', 'base': '.'}))"
  OUTPUT_VARIABLE PYTHON_INSTALL_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_subdirectory(lib)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()

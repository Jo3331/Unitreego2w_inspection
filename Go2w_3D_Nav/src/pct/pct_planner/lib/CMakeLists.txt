cmake_minimum_required(VERSION 3.13.0)
project(planning_core CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native") # have to enable because of gtsam

# GTSAM 4.2 and OSQP 1.0 have no Ubuntu 20.04 / Foxy packages, so they are built
# from source into a dependency prefix. Override GO2_DEPS_ROOT to relocate it.
# RPATH is enabled below, so no LD_LIBRARY_PATH is needed at run time.
if(NOT DEFINED GO2_DEPS_ROOT)
  set(GO2_DEPS_ROOT "$ENV{HOME}/go2_deps/install" CACHE PATH "Prefix holding the built GTSAM/OSQP/small_gicp")
endif()
list(APPEND CMAKE_PREFIX_PATH
  "${GO2_DEPS_ROOT}/gtsam-4.2"
  "${GO2_DEPS_ROOT}/osqp-1.0.0")

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Emit DT_RPATH instead of DT_RUNPATH. libgtsam.so itself depends on its
# sibling libmetis-gtsam.so but carries no RPATH of its own, and DT_RUNPATH is
# not consulted for a dependency's own dependencies. With DT_RPATH the search
# path of the loading object applies transitively, so the GTSAM install prefix
# added by CMAKE_INSTALL_RPATH_USE_LINK_PATH resolves libmetis-gtsam.so without
# needing LD_LIBRARY_PATH at run time.
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")

add_subdirectory(src/common)
add_subdirectory(src/a_star)
add_subdirectory(src/trajectory_optimization)
add_subdirectory(src/map_manager)
add_subdirectory(src/ele_planner)

# Install pybind11 modules to the python package directory
install(TARGETS 
  a_star 
  traj_opt 
  ele_planner 
  py_map_manager
  a_star_search
  gpmp_optimizer
  map_manager
  ele_planner_lib
  common_smoothing
  DESTINATION ${PYTHON_INSTALL_DIR}/pct_planner/lib
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_INSTALL_DIR}/pct_planner/lib)

# Install python stubs
install(FILES
  src/a_star/a_star.pyi
  src/trajectory_optimization/traj_opt.pyi
  src/ele_planner/ele_planner.pyi
  src/map_manager/py_map_manager.pyi
  DESTINATION ${PYTHON_INSTALL_DIR}/pct_planner/lib
  OPTIONAL
)

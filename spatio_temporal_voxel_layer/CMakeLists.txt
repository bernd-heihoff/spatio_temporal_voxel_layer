cmake_minimum_required(VERSION 3.5)
project(spatio_temporal_voxel_layer)

# Silence CMake dev warning about <PackageName>_ROOT variables (e.g. PCL_ROOT)
# and make find_package() honor them when set.
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Silence CMake dev warning from ament about option() clearing normal variables
# (BUILD_SHARED_LIBS) and make option() honor normal variables.
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get a linker error when there are undefined symbols
add_link_options(-Wl,--no-undefined)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(openvdb_vendor REQUIRED)

remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PCL REQUIRED COMPONENTS filters common)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

set(core_dependencies
  openvdb_vendor
)

set(bridge_dependencies
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  laser_geometry
  message_filters
  pcl_conversions
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
  builtin_interfaces
)

set(plugin_dependencies
  nav2_costmap_2d
  pluginlib
  ${bridge_dependencies}
)

add_definitions(${EIGEN3_DEFINITIONS})

set(srv_files "srv/SaveGrid.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

include_directories(
    include
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${nav2_costmap_2d_INCLUDE_DIRS}
)

set(stvl_core_sources
  src/spatio_temporal_voxel_grid.cpp
  src/frustum_models/depth_camera_frustum.cpp
  src/frustum_models/three_dimensional_lidar_frustum.cpp
)

add_library(stvl_core SHARED ${stvl_core_sources})

target_link_libraries(stvl_core
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenVDB::openvdb
)

ament_target_dependencies(stvl_core ${core_dependencies})

set(stvl_plugin_sources
  src/spatio_temporal_voxel_layer.cpp
  src/internal/observation_manager.cpp
)

add_library(stvl_bridge SHARED
  src/measurement_buffer.cpp
  src/internal/pruning_manager.cpp
  src/internal/sliding_window_2d.cpp
  src/internal/elevation_lethal.cpp
)

target_link_libraries(stvl_bridge
  stvl_core
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(stvl_bridge ${bridge_dependencies})

add_library(stvl_plugin SHARED ${stvl_plugin_sources})

target_link_libraries(stvl_plugin
  stvl_core
  stvl_bridge
  "${cpp_typesupport_target}"
  ${nav2_costmap_2d_LIBRARIES}
)

target_compile_definitions(stvl_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(stvl_plugin ${plugin_dependencies})

add_library(stvl_vdb2pc SHARED src/vdb2pc.cpp)

target_link_libraries(stvl_vdb2pc
  ${PCL_LIBRARIES}
  OpenVDB::openvdb
)

ament_target_dependencies(stvl_vdb2pc openvdb_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pruning_manager_test test/pruning_manager_test.cpp)
  if(TARGET pruning_manager_test)
    target_link_libraries(pruning_manager_test stvl_bridge)
    ament_target_dependencies(pruning_manager_test ${bridge_dependencies})
  endif()

  ament_add_gtest(measurement_buffer_test test/measurement_buffer_test.cpp)
  if(TARGET measurement_buffer_test)
    target_link_libraries(measurement_buffer_test stvl_bridge)
    ament_target_dependencies(measurement_buffer_test ${bridge_dependencies})
  endif()

  ament_add_gtest(sliding_window_2d_test test/sliding_window_2d_test.cpp)
  if(TARGET sliding_window_2d_test)
    target_link_libraries(sliding_window_2d_test stvl_bridge)
    ament_target_dependencies(sliding_window_2d_test ${bridge_dependencies})
  endif()

  ament_add_gtest(heartbeat_evaluator_test test/heartbeat_evaluator_test.cpp)
  if(TARGET heartbeat_evaluator_test)
    ament_target_dependencies(heartbeat_evaluator_test rclcpp)
  endif()

  ament_add_gtest(camera_info_fov_test test/camera_info_fov_test.cpp)
  if(TARGET camera_info_fov_test)
    target_link_libraries(camera_info_fov_test stvl_plugin)
    ament_target_dependencies(camera_info_fov_test ${plugin_dependencies})
  endif()
endif()

install(TARGETS stvl_core stvl_bridge stvl_plugin stvl_vdb2pc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY example
  DESTINATION share/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime ${plugin_dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(stvl_core stvl_bridge stvl_plugin stvl_vdb2pc)
ament_package()

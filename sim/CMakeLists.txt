# The simulation program: a ROS 2 node for Cosys-AirSim (localization_core
# airsim/). Built where ROS 2 Jazzy is sourced (the ros2
# distrobox), with Cosys-AirSim's ROS 2 workspace (airsim_interfaces) and
# yolo_ros's (yolo_msgs, build-ros/colcon) sourced too; sim/run_pole.sh does
# that.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(yolo_msgs REQUIRED)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

# The parameters: declared, described and checked by C++ that
# generate_parameter_library (PickNik) writes from the YAML file.
# Its CMake macro ends with ament_export_dependencies, which only matters
# for installed packages; this is a plain CMake build.
if(NOT COMMAND ament_export_dependencies)
  macro(ament_export_dependencies)
  endmacro()
endif()
generate_parameter_library(pole_tracking_sim_parameters src/pole_tracking_sim_parameters.yaml)
set_target_properties(pole_tracking_sim_parameters PROPERTIES SYSTEM TRUE) # no warnings from generated code

add_executable(pole_tracking_sim src/pole_tracking_sim.cpp)
target_include_directories(pole_tracking_sim PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(pole_tracking_sim PRIVATE pole_estimator localization_core_sim pole_tracking_sim_parameters
                      rclcpp::rclcpp tf2_ros::tf2_ros tf2_eigen::tf2_eigen
                      ${GeographicLib_LIBRARIES}
                      ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS}
                      ${airsim_interfaces_TARGETS}
                      ${yolo_msgs_TARGETS})

cmake_minimum_required(VERSION 3.22)

project(drone_localization VERSION 0.1.0 LANGUAGES CXX)

set(gnss_imu_fusion_dir ${CMAKE_CURRENT_SOURCE_DIR}/external/gnss-imu-fusion)
if(NOT EXISTS ${gnss_imu_fusion_dir}/CMakeLists.txt)
  message(FATAL_ERROR "external/gnss-imu-fusion is empty, run: git submodule update --init")
endif()
include(${gnss_imu_fusion_dir}/cmake/common.cmake)
add_subdirectory(${gnss_imu_fusion_dir})

# The filter fusing uwb ranges, camera rays, imu and gps (header only).
add_library(pole_estimator INTERFACE)
target_include_directories(pole_estimator
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/sim/src)
target_link_libraries(pole_estimator INTERFACE gnss_imu_fusion_headers)

enable_testing()
add_subdirectory(test/cpp)

# Gazebo simulation bridge, see sim/run_pole.sh.
find_package(gz-transport13 QUIET)
find_package(gz-msgs10 QUIET)
if(gz-transport13_FOUND AND gz-msgs10_FOUND)
  add_subdirectory(sim)
endif()

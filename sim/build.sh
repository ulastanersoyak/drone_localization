#!/usr/bin/env bash
# Builds the simulation side in the "ros2" distrobox ($LOCSIM_ROS_BOX):
# yolo_ros's packages (external/yolo_ros: yolo_msgs and its nodes) into
# build-ros/colcon with colcon, then pole_tracking_sim into build-ros with
# CMake. Both are incremental. run_pole.sh and detector/train.sh call it.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
core=${LOCALIZATION_CORE_DIR:-$root/external/localization_core}
core=$(cd "$core" && pwd)
box=${LOCSIM_ROS_BOX:-ros2}
if [[ ! -f $root/external/yolo_ros/yolo_msgs/package.xml ]]; then
  echo "external/yolo_ros is empty, run: git submodule update --init" >&2
  exit 1
fi
colcon=$root/build-ros/colcon
distrobox enter "$box" -- bash -c "
  set -e; . /opt/ros/jazzy/setup.bash; . '$core/external/Cosys-AirSim/ros2/install/setup.bash'
  cd '$root'
  colcon --log-base '$colcon/log' build --base-paths external/yolo_ros --packages-select yolo_msgs yolo_ros \
    --build-base '$colcon/build' --install-base '$colcon/install' --event-handlers console_cohesion- >/dev/null
  . '$colcon/install/setup.bash'
  cmake -S '$root' -B '$root/build-ros' -DCMAKE_BUILD_TYPE=Release -DLOCALIZATION_CORE_DIR='$core' >/dev/null
  cmake --build '$root/build-ros' -j --target pole_tracking_sim"

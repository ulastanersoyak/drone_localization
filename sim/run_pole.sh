#!/usr/bin/env bash
# Flies the drone in Cosys-AirSim (Unreal) beside a pole carrying a UWB anchor
# and a pan-tilt camera, and runs pole_tracking_sim, which fuses UWB, camera
# detections, IMU and GPS, against it over ROS 2.
#
#   sim/run_pole.sh [--scenario field|antwerp] [--detector yolo|synthetic]
#                   [--no-gui] [--rviz] [weather] [options]
#
#   field      (default) open ground
#   antwerp    the same pole and flight on Groenplaats, among Antwerp's buildings
#   yolo       (default) yolo_ros (external/yolo_ros) on the camera images,
#              with weights trained on AirSim images (sim/detector/train.sh)
#   synthetic  sim/detector/synthetic_detect.py: the true position projected
#              into the camera, with noise and misses (no YOLO needed)
#   --rviz     rviz2 with sim/rviz/pole.rviz (the estimates, the true drone
#              frame, the camera image with YOLO's boxes)
#   weather    --rain I --snow I --fog I --dust I --leaves I (0..1), --wind SPEED[@FROM_DEG]
#   options    ROS parameters written --name value: pole_tracking_sim's
#              (--duration S, --no-gps, --no-imu, --uwb-noise M, ...; the
#              list: sim/src/pole_tracking_sim_parameters.yaml) and the
#              autopilot's (--radius M, --altitude M, --altitude-swing M)
#
# The simulator side is localization_core/airsim (run airsim/setup.sh once);
# ROS 2 and the detector run in the "ros2" distrobox, started by
# sim/launch/pole_tracking.launch.py. AirSim's ROS node publishes the pole
# camera (/airsim_node/drone/pole_Scene/image) and turns it.
# The drone is flown by localization_core's airsim/autopilot.py.
#
# Each run is recorded into build-ros/runs/pole-<scenario>-<time> (rosbag2:
# the truth as TF, the estimates, the detections) and scored at the end by
# localization_core airsim/evaluate.py (evo); evo's own tools plot it.
# LOCALIZATION_CORE_DIR=../localization_core uses a side by side checkout.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
core=${LOCALIZATION_CORE_DIR:-$root/external/localization_core}
core=$(cd "$core" 2>/dev/null && pwd || true)
if [[ ! -x $core/airsim/run.sh ]]; then
  echo "localization_core with airsim/ not found at ${LOCALIZATION_CORE_DIR:-external/localization_core}," \
    "run: git submodule update --init (or set LOCALIZATION_CORE_DIR)" >&2
  exit 1
fi
box=${LOCSIM_ROS_BOX:-ros2}
# shellcheck source=/dev/null
. "$core/airsim/params.sh"

scenario=field
detector=yolo
engine_args=()
launch_args=()
args=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --scenario) scenario=$2; shift 2 ;;
    --detector) detector=$2; shift 2 ;;
    --no-gui|--headless) engine_args+=(--offscreen); shift ;;
    --rviz) launch_args+=(rviz:=true); shift ;;
    --rain|--snow|--fog|--dust|--leaves|--wind) engine_args+=("$1" "$2"); shift 2 ;;
    *) args+=("$1"); shift ;;
  esac
done
case $scenario in
  field) map=Field ;;
  antwerp) map=Antwerp ;;
  *) echo "unknown scenario $scenario (field, antwerp)" >&2; exit 2 ;;
esac
settings=$core/airsim/settings/pole_$scenario.json

weights=$root/build-sim/drone-airsim.pt
if [[ $detector == yolo && ! -f $weights ]]; then
  echo "no YOLO weights for AirSim images ($weights): train them with" \
    "sim/detector/train.sh, or run with --detector synthetic" >&2
  exit 1
fi

# Build yolo_ros (its messages are pole_tracking_sim's input) and
# pole_tracking_sim where ROS 2 and AirSim's ROS messages are.
"$root/sim/build.sh"

to_ros_params ${args[@]+"${args[@]}"}
bag=$root/build-ros/runs/pole-$scenario-$detector-$(date +%Y%m%d-%H%M%S)
mkdir -p "$(dirname "$bag")"

# Unreal, then sim/launch/pole_tracking.launch.py (AirSim's ROS node, the
# detector, pole_tracking_sim, the recorder); the run ends when
# pole_tracking_sim does.
status=0
LOCSIM_ROS_SETUP=$root/build-ros/colcon/install/setup.bash \
  "$core/airsim/run.sh" --map "$map" --settings "$settings" ${engine_args[@]+"${engine_args[@]}"} \
  --launch "$root/sim/launch/pole_tracking.launch.py" "settings:=$settings" "detector:=$detector" \
  "weights:=$weights" "imgsz:=${DETECTOR_IMGSZ:-1280}" "record:=$bag" \
  "params:=${ros_params[*]+${ros_params[*]}}" ${launch_args[@]+"${launch_args[@]}"} || status=$?

if [[ -d $bag ]]; then
  # evo reads the TF from the recording with ROS's tf2_py.
  distrobox enter "$box" -- bash -c ". /opt/ros/jazzy/setup.bash && exec '$core/airsim/.venv/bin/python' \
    '$core/airsim/evaluate.py' '$bag' 'fused (UWB + camera + IMU + GPS)=/pole/fused' \
    'camera ray + UWB only=/pole/ray_uwb' 'raw GPS=/pole/gps'" || true
fi
exit "$status"

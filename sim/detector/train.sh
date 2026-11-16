#!/usr/bin/env bash
# Generates a labelled image set from simulated flights in Cosys-AirSim and
# trains the drone detector (ultralytics YOLO). The labels come from the true
# pose, so nothing is labelled by hand. Needs sim/detector/setup.sh first (the
# YOLO environment) and localization_core/airsim/setup.sh (the simulator).
#
#   sim/detector/train.sh [epochs]        -> build-sim/drone-airsim.pt
#
# Each flight is one AirSim run (localization_core/airsim/run.sh with
# sim/launch/pole_tracking.launch.py, no detector) with the pole camera at rest
# (pole_tracking_sim's fixed_camera, so the drone crosses the whole frame),
# in the open field and in Antwerp, in AirSim's weather (rain, snow, fog): a
# detector trained in one light and place fails in another. make_dataset.py
# records frames with AirSim's own 2D box of the drone as the label. Flights
# run one after the other; ROS 2 and the Python steps run in the "ros2"
# distrobox.
#
#   YOLO_WORKERS=2        data loader processes; each takes about 1 GB of RAM
#                         at imgsz 1280 (the default 8 filled a 16 GB machine)
#   LOCSIM_REUSE_DATA=1   train on the existing dataset, skip the flights
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
core=${LOCALIZATION_CORE_DIR:-$root/external/localization_core}
core=$(cd "$core" && pwd)
box=${LOCSIM_ROS_BOX:-ros2}
epochs=${1:-60}
workers=${YOLO_WORKERS:-2}
reuse=${LOCSIM_REUSE_DATA:-0}
data=$root/build-sim/yolo-data-airsim
weights=$root/build-sim/drone-airsim.pt

# name split scenario radius altitude swing [weather options of airsim/run.sh]
flights=("fa train field 8 8 3"
         "fb train field 6 6 2 --rain 0.5"
         "fc train field 10 10 4 --rain 0.9 --fog 0.3 --wind 8@270"
         "fd train field 8 7 3 --snow 0.6"
         "fe train field 9 8 3 --fog 0.2"
         "ca train antwerp 8 8 3"
         "cb train antwerp 6 6 2 --rain 0.6"
         "cc train antwerp 10 9 3 --snow 0.5"
         "fv val field 9 7 3 --rain 0.7 --wind 6@200"
         "cv val antwerp 7 7 2 --fog 0.3")

in_box () { distrobox enter "$box" -- bash -c "$1"; }

"$root/sim/build.sh"
if [[ $reuse == 1 && -f $data/drone.yaml ]]; then
  echo "reusing $data"
  flights=()
else
  rm -rf "$data"
fi
mkdir -p "$data"

run=
cleanup () { [[ -n $run ]] && kill "$run" 2>/dev/null && wait "$run" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for flight in ${flights[@]+"${flights[@]}"}; do
  # shellcheck disable=SC2086
  set -- $flight
  name=$1 split=$2 scenario=$3 radius=$4 altitude=$5 swing=$6
  shift 6
  case $scenario in field) map=Field ;; antwerp) map=Antwerp ;; esac
  settings=$core/airsim/settings/pole_$scenario.json
  echo "flight $name ($split): $scenario, radius $radius, altitude $altitude +- $swing${1:+, $*}"
  # Unreal and the pole tracking launch file without a detector, the camera
  # at rest; it ends after --duration (stopped earlier below if need be).
  LOCSIM_ROS_SETUP=$root/build-ros/colcon/install/setup.bash \
    "$core/airsim/run.sh" --map "$map" --settings "$settings" --offscreen "$@" \
    --launch "$root/sim/launch/pole_tracking.launch.py" "settings:=$settings" detector:=none \
    "params:=fixed_camera:=true wait_for_detector:=false duration:=110 seed:=7 radius:=$radius altitude:=$altitude altitude_swing:=$swing" \
    >"$data/flight_$name.log" 2>&1 &
  run=$!
  until (exec 3<>/dev/tcp/127.0.0.1/41451) 2>/dev/null; do sleep 1; done
  sleep 10  # take-off
  in_box "exec '$core/airsim/.venv/bin/python' '$root/sim/detector/make_dataset.py' --out '$data/$split' \
    --prefix $name --seconds 90" || true
  kill "$run" 2>/dev/null; wait "$run" 2>/dev/null || true
  run=
done

cat >"$data/drone.yaml" <<YAML
train: $data/train/images
val: $data/val/images
names:
  0: drone
YAML

runs=$root/build-sim/yolo-runs
name=drone-airsim
in_box ". '$root/sim/detector/env.sh' && cd '$root/build-sim' && yolo detect train model=yolo11n.pt \
  data='$data/drone.yaml' epochs=$epochs imgsz=1280 batch=6 workers=$workers patience=20 mosaic=0.5 \
  fliplr=0.5 flipud=0 degrees=0 project='$runs' name=$name exist_ok=True verbose=False"
cp "$runs/$name/weights/best.pt" "$weights"
echo "weights: $weights"

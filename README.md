# drone_localization

locates a quadrotor in 3d. the current setup is a pole beside its flight
path carrying a uwb anchor (range to a tag on the drone) and a camera on a
pan-tilt mount whose images go through a yolo detector (bearing to the
drone). a kalman filter fuses range and bearing, optionally with the drone's
own imu and gps, into a 3d position. everything runs in cosys-airsim (unreal
engine) over ros 2 and is scored against the true pose.

the kalman core, the imu sample type and the simulator side come from
[localization_core](external/localization_core), included as a git submodule. its sibling project, `gnss_imu_prediction`, estimates and
predicts the drone's position from its own gps and imu.

guides: [getting started](https://ulastanersoyak.github.io/drone_localization/getting-started.html) ·
[how it works](https://ulastanersoyak.github.io/drone_localization/how-it-works.html)

## layout

| path | what |
|---|---|
| `sim/src/pole_estimator.hpp` | the filter (header only, target `pole_estimator`) and the pole camera model |
| `sim/src/pole_tracking_sim.cpp` | the ros 2 node: simulated uwb, detections → rays, pan-tilt steering |
| `sim/launch/pole_tracking.launch.py` | starts airsim's ros side, the detector, the program and the recorder |
| `sim/detector/` | yolo setup, dataset generation and training, the synthetic detector |
| `sim/run_pole.sh`, `sim/build.sh` | build and run a simulation |
| `test/cpp/` | unit tests of the estimator |
| `external/localization_core/` | submodule: shared libraries, the simulator side (`airsim/`) |
| `external/yolo_ros/` | submodule: [yolo_ros](https://github.com/mgonzs13/yolo_ros), the live detector |

## building

needs cmake ≥ 3.22, a c++20 compiler, eigen and geographiclib (fedora:
`eigen3-devel GeographicLib-devel`, ubuntu: `libeigen3-dev
libgeographiclib-dev`); the simulation needs the `ros2` distrobox set up by
localization_core's `airsim/setup.sh` (see below).

```
git clone --recurse-submodules <url>/drone_localization
cd drone_localization        # or, in an existing clone: git submodule update --init
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

for local clones side by side, git refuses `file://` submodules by default:
`git -c protocol.file.allow=always submodule update --init`.

## simulation

the same quadrotor is tracked from a pole beside its flight path. the pole
carries a uwb anchor (range, 20 hz) and a camera on a pan-tilt mount. a yolo
detector finds the drone in the camera image and each detection becomes a
bearing ray. `pole_tracking_sim` fuses the ranges and rays, optionally with
the drone's own imu and gps; the run is recorded and the estimate scored
against the true pose.
the camera is turned towards the estimate (never the ground truth), slew
rate limited, so it keeps the drone in view. detections arrive late because
yolo takes time, so all measurements are buffered and fused in timestamp
order a fixed time behind the simulation.

the simulation runs in [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim)
(Unreal Engine) over ROS 2: airsim flies the drone and simulates its imu and
gps, renders the pole camera (an airsim "external" camera, published and
turned by airsim's own ros 2 node), and `pole_tracking_sim` and the detector
([yolo_ros](https://github.com/mgonzs13/yolo_ros) with our weights) are ros 2
nodes, started by `sim/launch/pole_tracking.launch.py`. where the pole stands
and how its camera sees come from airsim's settings
(`localization_core/airsim/settings/pole_*.json`). the simulator side lives in
localization_core: run `external/localization_core/airsim/setup.sh` once (see
[localization_core/airsim](external/localization_core/airsim/README.md)).

the detector is trained once on airsim images from simulated flights in the
open field and in antwerp, in airsim's weather, labelled with airsim's own
object detection (the drone's box as unreal sees it):

```
distrobox enter ros2 -- sim/detector/setup.sh     # yolo into build-sim/yolo-venv (once)
sim/detector/train.sh                             # dataset + training, writes build-sim/drone-airsim.pt
```

then:

```
sim/run_pole.sh                                   # the open field, unreal window
sim/run_pole.sh --scenario antwerp                # the same pole on groenplaats, antwerp
sim/run_pole.sh --no-gui --duration 120           # no window
sim/run_pole.sh --detector synthetic              # no yolo: the true position projected, with noise
sim/run_pole.sh --rain 0.6 --fog 0.2              # airsim's weather
sim/run_pole.sh --no-gps --no-imu                 # camera and uwb only
```

`run_pole.sh` builds yolo_ros into `build-ros/colcon` and `pole_tracking_sim`
into `build-ros/` in the `ros2` distrobox (ros 2 jazzy; the yolo environment
is shared with it; `sim/build.sh`), starts unreal and then the launch file:
airsim's ros 2 node, the detector, the program, which prints the fused error
every second, and a recorder. the run is recorded into
`build-ros/runs/pole-<scenario>-<detector>-<time>` and scored at the end by
localization_core's `airsim/evaluate.py` (evo): the fused estimate
(`/pole/fused`), the camera ray at the uwb distance (`/pole/ray_uwb`) and
the raw gps (`/pole/gps`) against the true pose. the detector logs to the
ros log (`~/.ros/log/latest/`). watch with rviz2 (fixed frame `enu`,
`/pole/markers`) and `rqt_image_view /pole/dbg_image`. `DETECTOR_IMGSZ` sets the yolo
input width (default 1280). the options are `pole_tracking_sim`'s ros 2
parameters (`--uwb-noise 0.2` becomes `uwb_noise:=0.2`):

| option | default | |
|---|---|---|
| `--no-gps` | | don't use the gps |
| `--no-imu` | | don't use the imu |
| `--uwb-noise m` | 0.1 | uwb range noise |
| `--gps-noise m` | 1.5 | white gps position noise |
| `--latency s` | 0.4 | fuse measurements this far behind the simulation |
| `--duration s` | forever | stop after this much simulated time |
| `--radius m` | 8 | figure-eight loop radius |
| `--altitude m` | 8 | cruise height above the ground |
| `--altitude-swing m` | 3 | how far it climbs and descends around the cruise height |
| `--seed n` | 1 | noise seed |
| `--fixed-camera` | | keep the pole camera in its rest pose instead of turning it towards the drone |
| `--slew-rate deg` | 120 | pan-tilt speed limit, degrees per second |

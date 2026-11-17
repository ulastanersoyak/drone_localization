"""pole_tracking_sim against Cosys-AirSim: the simulator's ROS side
(localization_core airsim/launch/airsim.launch.py: AirSim's node, which also
publishes the pole camera and turns it), a drone detector and
pole_tracking_sim. Unreal must be running; sim/run_pole.sh starts both
through localization_core's airsim/run.sh.

    ros2 launch pole_tracking.launch.py settings:=.../pole_field.json \\
        [detector:=yolo|synthetic|none] [weights:=build-sim/drone-airsim.pt]
        [params:="duration:=60 no_gps:=true"] [record:=DIR] [rain:=0.5 ...]

    detector  yolo: yolo_ros (external/yolo_ros, built into build-ros/colcon)
              with our weights, plus its debug node (/pole/dbg_image, the
              boxes drawn on the camera image); synthetic:
              sim/detector/synthetic_detect.py (the true position
              projected, with noise); none: no detector (sim/detector/train.sh records
              images instead)
    params    pole_tracking_sim's ROS parameters, NAME:=VALUE separated by
              spaces (`ros2 param describe /pole_tracking_sim <name>`)
    record    a rosbag2 directory: the truth (TF), the estimates and the
              detections, for localization_core airsim/evaluate.py

Needs yolo_ros's workspace sourced (yolo_msgs): run_pole.sh passes it to
airsim/run.sh in LOCSIM_ROS_SETUP. The run ends when pole_tracking_sim does
(duration, Ctrl-C). The detector's output goes to the ROS log
(~/.ros/log/latest/).
"""
import json
import math
import os
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AIRSIM = os.environ.get("LOCSIM_AIRSIM_DIR", os.path.join(ROOT, "external", "localization_core", "airsim"))
DETECTOR = os.path.join(ROOT, "sim", "detector")
CAMERA_IMAGE = "/airsim_node/drone/pole_Scene/image"
TOPICS = ["/pole/fused", "/pole/ray_uwb", "/pole/gps", "/pole/detections"]
# yolo_ros's nodes in the YOLO environment (torch, ultralytics, the host's
# NVIDIA driver): sim/detector/env.sh, then its Python runs the node script.
YOLO_ENV = f"bash -c '. {DETECTOR}/env.sh && exec python \"$0\" \"$@\"'"


def yolo(weights, imgsz):
    # The camera is 1920 x 1080: imgsz across, the height in proportion,
    # rounded up to YOLO's stride of 32 (what imgsz=1280 letterboxes to).
    height = math.ceil(imgsz * 1080 / 1920 / 32) * 32
    common = dict(package="yolo_ros", namespace="pole", prefix=YOLO_ENV,
                  remappings=[("image_raw", CAMERA_IMAGE)], output="log")
    return [
        Node(executable="yolo_node", name="yolo", **common,
             parameters=[{"model": weights, "device": "cuda:0", "threshold": 0.25,
                          "imgsz_width": imgsz, "imgsz_height": height}]),
        Node(executable="debug_node", name="yolo_debug", **common),
    ]


def programs(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    actions = []

    detector = arg("detector")
    if detector == "yolo":
        actions += yolo(arg("weights"), int(arg("imgsz")))
    elif detector == "synthetic":
        actions.append(ExecuteProcess(
            cmd=["python3", os.path.join(DETECTOR, "synthetic_detect.py")],
            name="synthetic_detect", output="log"))
    elif detector != "none":
        raise RuntimeError(f"unknown detector {detector} (yolo, synthetic, none)")

    # The enu frame's origin: AirSim's OriginGeopoint (its GPS reference).
    o = json.load(open(arg("settings")))["OriginGeopoint"]
    origin = {"origin_latitude": float(o["Latitude"]), "origin_longitude": float(o["Longitude"]),
              "origin_altitude": float(o["Altitude"])}
    overrides = [a for p in shlex.split(arg("params")) for a in ("-p", p)]
    sim = Node(executable=os.path.join(ROOT, "build-ros", "sim", "pole_tracking_sim"), name="pole_tracking_sim",
               parameters=[origin], ros_arguments=overrides, output="screen", emulate_tty=True)
    actions += [sim, RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("settings", description="the AirSim settings.json Unreal runs with"),
        DeclareLaunchArgument("detector", default_value="yolo", description="yolo, synthetic or none"),
        DeclareLaunchArgument("weights", default_value=os.path.join(ROOT, "build-sim", "drone-airsim.pt")),
        DeclareLaunchArgument("imgsz", default_value="1280", description="YOLO input width"),
        DeclareLaunchArgument("params", default_value="", description="pole_tracking_sim parameters, NAME:=VALUE ..."),
        DeclareLaunchArgument("record", default_value="", description="rosbag2 directory to record into"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(AIRSIM, "launch", "airsim.launch.py")),
                                 launch_arguments={"record": LaunchConfiguration("record"),
                                                   "record_topics": " ".join(TOPICS)}.items()),
        OpaqueFunction(function=programs),
    ])

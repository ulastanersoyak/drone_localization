#!/usr/bin/env python3
"""A stand-in for YOLO: the drone's true position projected into the pole
camera, with pixel noise, misses and a processing delay, published like
yolo_ros publishes its detections.

For runs without the YOLO environment or weights, and to separate detector
errors from fusion errors: a detector that is right up to its pixel noise.
A ROS 2 node: each camera pose AirSim's ROS node publishes with an image
(/airsim_node/drone/pole_Scene/pose) is a frame. The pose goes into tf2 as
world -> pole_camera, so tf2 gives the drone (the TF ... -> drone AirSim
publishes, the truth) in camera coordinates (x forward, y left, z up) at
the frame's time; the pinhole from AirSim's camera_info makes it a pixel.
The detection keeps the frame's stamp, as yolo_ros does.

    synthetic_detect.py [--pixel-noise 2] [--miss-rate 0.05] [--delay 0.08]

Runs with yolo_ros's workspace sourced (yolo_msgs), as
pole_tracking.launch.py does.
"""
import argparse
import random
import threading
import time
from collections import deque

import rclpy
import rclpy.executors
import rclpy.time
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from yolo_msgs.msg import Detection, DetectionArray

CAMERA_FRAME = "pole_camera"
TOPIC = "/airsim_node/drone/pole_Scene/"


def spin_quietly(node):
    """rclpy.spin, without a traceback when the node is shut down from outside."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pixel-noise", type=float, default=2.0, help="sigma, pixels")
    ap.add_argument("--miss-rate", type=float, default=0.05)
    ap.add_argument("--delay", type=float, default=0.08, help="processing time, wall clock seconds")
    ap.add_argument("--seed", type=int, default=3)
    args, ros_args = ap.parse_known_args()

    rng = random.Random(args.seed)
    lock = threading.Lock()
    frames = deque()  # camera frames waiting for the truth at their time
    pending = deque()  # (publish at wall time, detections)
    camera = {}  # intrinsics from camera_info
    count = {"frames": 0, "detections": 0}

    rclpy.init(args=ros_args)
    node = Node("synthetic_detect")
    tf = Buffer()
    TransformListener(tf, node)
    pub = node.create_publisher(DetectionArray, "/pole/detections", 20)

    def on_info(msg):
        with lock:
            camera.update(f=msg.k[0], cu=msg.k[2], cv=msg.k[5], width=msg.width, height=msg.height)

    def on_frame(msg):
        t = TransformStamped()
        t.header, t.child_frame_id = msg.header, CAMERA_FRAME
        p = msg.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = msg.pose.orientation
        tf.set_transform(t, "synthetic_detect")
        with lock:
            frames.append((msg.header, time.time()))

    def detect(header):
        """True if the frame is done (detected or not), False to retry later."""
        try:
            d = tf.lookup_transform(CAMERA_FRAME, "drone", rclpy.time.Time.from_msg(header.stamp))
        except TransformException:
            return False  # the truth for that moment has not arrived yet
        x, y, z = d.transform.translation.x, d.transform.translation.y, d.transform.translation.z
        with lock:
            c = dict(camera)
        if not c or x <= 0.1 or rng.random() < args.miss_rate:
            return True
        u = c["cu"] - c["f"] * y / x + rng.gauss(0, args.pixel_noise)
        v = c["cv"] - c["f"] * z / x + rng.gauss(0, args.pixel_noise)
        if not (0 <= u < c["width"] and 0 <= v < c["height"]):
            return True
        out = DetectionArray()
        out.header = header  # the frame's time
        det = Detection()
        det.class_id, det.class_name, det.score = 0, "drone", 0.9
        det.bbox.center.position.x, det.bbox.center.position.y = u, v
        out.detections.append(det)
        pending.append((time.time() + args.delay, out))
        return True

    node.create_subscription(CameraInfo, TOPIC + "camera_info", on_info, 10)
    node.create_subscription(PoseStamped, TOPIC + "pose", on_frame, 20)
    spinner = threading.Thread(target=spin_quietly, args=(node,), daemon=True)
    spinner.start()
    print("synthetic detector: the true position projected, %.1f px noise, %.0f %% misses" %
          (args.pixel_noise, 100 * args.miss_rate), flush=True)
    report = time.time()
    try:
        while rclpy.ok():
            now = time.time()
            with lock:
                waiting = list(frames)
                frames.clear()
            for header, arrived in waiting:
                if detect(header):
                    count["frames"] += 1
                elif now - arrived < 1.0:
                    with lock:
                        frames.append((header, arrived))
            while pending and pending[0][0] <= now:
                pub.publish(pending.popleft()[1])
                count["detections"] += 1
            if now - report > 10:
                report = now
                print("synthetic detector: %d frames, %d detections" % (count["frames"], count["detections"]), flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    try:
        rclpy.try_shutdown()
    except RuntimeError:
        pass  # rclpy's signal handler shut the context down in between
    spinner.join(timeout=2)  # exit only once the ROS thread has


if __name__ == "__main__":
    main()

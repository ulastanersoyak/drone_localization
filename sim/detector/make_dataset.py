#!/usr/bin/env python3
"""Saves pole camera frames with YOLO labels from AirSim's object detection.

Runs next to a simulation (sim/detector/train.sh does): it pauses AirSim,
takes a pole camera image and AirSim's 2D box of the drone for that same
moment (simGetDetections: the box Unreal computes from the drone's model),
resumes, and writes the image and the label. Pausing keeps image and box
together; nothing is projected or measured here.

    make_dataset.py --out DIR [--prefix f] [--seconds 90] [--interval 0.15]

Needs AirSim's Python client (localization_core airsim/.venv).
"""
import argparse
import pathlib
import time

import cosysairsim as airsim
import cv2
import numpy as np

from airsim_detections import CAMERA, SCENE, VEHICLE, connect, drone_box


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, type=pathlib.Path)
    ap.add_argument("--prefix", default="f")
    ap.add_argument("--seconds", type=float, default=90, help="wall clock seconds to record")
    ap.add_argument("--interval", type=float, default=0.15, help="seconds between frames")
    ap.add_argument("--max-range", type=float, default=80, help="metres; farther drones are not labelled")
    args = ap.parse_args()
    (args.out / "images").mkdir(parents=True, exist_ok=True)
    (args.out / "labels").mkdir(parents=True, exist_ok=True)

    client = connect(args.max_range)

    saved = labelled = 0
    end = time.time() + args.seconds
    while time.time() < end:
        client.simPause(True)
        try:
            response = client.simGetImages([airsim.ImageRequest(CAMERA, SCENE, False, False)], vehicle_name=VEHICLE)[0]
            detections = client.simGetDetections(CAMERA, SCENE, vehicle_name=VEHICLE)
        finally:
            client.simPause(False)
        w, h = response.width, response.height
        if w == 0:
            continue
        # The bytes are RGB (AirSim on Vulkan); OpenCV writes BGR.
        img = np.frombuffer(response.image_data_uint8, np.uint8).reshape(h, w, 3)
        name = f"{args.prefix}_{saved:05d}"
        cv2.imwrite(str(args.out / "images" / f"{name}.jpg"), img[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 95])
        # Only drones fully in the frame: a cut box would teach YOLO wrong
        # extents.
        box = drone_box(detections, w, h, whole=True)
        with open(args.out / "labels" / f"{name}.txt", "w") as f:
            if box is not None:
                x0, y0, x1, y1 = box
                f.write("0 %.6f %.6f %.6f %.6f\n" % ((x0 + x1) / 2 / w, (y0 + y1) / 2 / h, (x1 - x0) / w, (y1 - y0) / h))
                labelled += 1
        saved += 1
        time.sleep(args.interval)
    print(f"make_dataset {args.prefix}: {saved} frames, {labelled} labelled", flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env bash
# Installs YOLO into build-sim/yolo-venv. Run once, inside the ros2 distrobox.
# The venv sees the system's ROS 2 Python packages; yolo_ros's nodes run in it
# (sim/launch/pole_tracking.launch.py) and use ROS's cv_bridge, which is
# built against NumPy 1.x: hence yolo_ros's pins (numpy<2) and an OpenCV
# that still supports them.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
sudo apt-get install -y -q python3-venv python3-pip
python3 -m venv --system-site-packages "$root/build-sim/yolo-venv"
. "$root/build-sim/yolo-venv/bin/activate"
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics 'numpy<2' 'opencv-python<4.12' 'contourpy<1.4' lap

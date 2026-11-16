"""AirSim's own 2D box of the drone in the pole camera (Cosys-AirSim's object
detection: Unreal projects the drone's mesh into the camera image), the
training labels of make_dataset.py. Needs AirSim's Python client
(localization_core airsim/.venv).
"""
import cosysairsim as airsim

CAMERA, VEHICLE = "pole", "drone"
SCENE = airsim.ImageType.Scene


def connect(max_range_m=80.0, camera=CAMERA):
    """A client with the detection filter set to the drone actor (its name in
    the settings) up to max_range_m."""
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.simClearDetectionMeshNames(camera, SCENE, vehicle_name=VEHICLE)
    client.simSetDetectionFilterRadius(camera, SCENE, max_range_m * 100, vehicle_name=VEHICLE)
    client.simAddDetectionFilterMeshName(camera, SCENE, VEHICLE, vehicle_name=VEHICLE)
    return client


def drone_box(detections, width, height, whole=True):
    """(x0, y0, x1, y1) in pixels of the drone among AirSim's detections, or
    None. whole: only a box entirely inside the image (a cut box has wrong
    extents)."""
    for d in detections:
        if d.name != VEHICLE:
            continue
        x0, y0, x1, y1 = d.box2D.min.x_val, d.box2D.min.y_val, d.box2D.max.x_val, d.box2D.max.y_val
        if x1 - x0 < 2 or y1 - y0 < 2:
            continue
        if whole and (x0 <= 0 or y0 <= 0 or x1 >= width - 1 or y1 >= height - 1):
            continue
        return x0, y0, x1, y1
    return None

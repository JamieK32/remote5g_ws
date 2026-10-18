"""Resolve installed vehicle geometry without assuming a workspace layout."""
from dataclasses import dataclass
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


@dataclass(frozen=True)
class VehicleGeometry:
    wheelbase: float
    camera_x: float
    camera_height: float
    camera_pitch: float
    max_speed: float


def load_vehicle_geometry(path=''):
    source = (Path(path).expanduser() if path else
              Path(get_package_share_directory('remote5g_sim'))/'config/vehicle.yaml')
    p = yaml.safe_load(source.read_text())
    geometry = VehicleGeometry(float(p['wheelbase']), float(p['front_x']),
                               float(p['wheel_radius'])+float(p['front_z']),
                               float(p['front_pitch']), float(p['max_speed']))
    if not all(math.isfinite(v) for v in vars(geometry).values()):
        raise ValueError(f'Non-finite vehicle geometry in {source}')
    if min(geometry.wheelbase, geometry.camera_height, geometry.max_speed) <= 0:
        raise ValueError(f'Invalid vehicle geometry in {source}')
    return geometry

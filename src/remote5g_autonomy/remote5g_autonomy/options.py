"""Typed application configuration; usable without ROS or a source checkout."""
import argparse
from dataclasses import asdict, dataclass
import math
from pathlib import Path

import yaml


MAX_REQUEST_SPEED = 2.1  # Preserve the user's current requested-speed range.


@dataclass
class FollowOptions:
    drive: bool = False
    show: bool = False
    filter_straight: bool = False
    continuous: bool = False
    reset_on_exit: bool = False
    speed: float = 0.10
    kp: float = 0.35
    max_steer: float = 0.22
    seconds: float = 15.0
    max_distance: float = 2.0
    lookahead: float = 2.5
    lane_width: float = 1.22
    vehicle_config: str = ''
    vehicle_entity: str = 'xtnetrc'
    save_dir: str = ''
    image_timeout: float = 0.6
    odom_timeout: float = 1.0
    required_good_frames: int = 3
    acquisition_timeout: float = 12.0
    loss_timeout: float = 0.5
    control_period: float = 0.02

    def validate(self):
        if self.reset_on_exit and not self.drive:
            raise ValueError('--reset-on-exit requires --drive; preview never resets the vehicle')
        for key, default in asdict(FollowOptions()).items():
            value = getattr(self, key)
            if isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError(f'{key} must be a boolean')
            elif isinstance(default, str):
                if not isinstance(value, str):
                    raise ValueError(f'{key} must be a string')
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{key} must be a finite number')
        if not self.vehicle_entity.strip():
            raise ValueError('--vehicle-entity must not be empty')
        bounds = {'speed': (0, MAX_REQUEST_SPEED), 'kp': (0, 2), 'max_steer': (0, 0.4),
                  'seconds': (0, 160), 'max_distance': (0, 180)}
        for key, (low, high) in bounds.items():
            if not low < getattr(self, key) <= high:
                raise ValueError(f'Require {low} < {key} <= {high}')
        if not 0.8 <= self.lookahead <= 6 or not 0.5 <= self.lane_width <= 3:
            raise ValueError('Require 0.8 <= lookahead <= 6 and 0.5 <= lane-width <= 3')
        for key in ('image_timeout', 'odom_timeout', 'acquisition_timeout', 'loss_timeout', 'control_period'):
            if getattr(self, key) <= 0:
                raise ValueError(f'{key} must be positive')
        if type(self.required_good_frames) is not int or self.required_good_frames < 1:
            raise ValueError('required_good_frames must be a positive integer')
        return self


def parse_args(argv=None, default_config=None):
    preliminary = argparse.ArgumentParser(add_help=False)
    preliminary.add_argument('--config', default=default_config)
    preliminary.add_argument('--vehicle-config')
    config, _ = preliminary.parse_known_args(argv)
    parser = argparse.ArgumentParser(description='ROS 2 visual lane following')
    parser.add_argument('--config', default=config.config, help='application YAML configuration')
    defaults = asdict(FollowOptions())
    if config.config:
        try:
            settings = yaml.safe_load(Path(config.config).expanduser().read_text()) or {}
            if not isinstance(settings, dict) or set(settings)-set(defaults):
                raise ValueError('config must be a mapping containing only FollowOptions fields')
            defaults.update(settings)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    descriptions = {
        'drive': 'enable vehicle commands; preview by default',
        'show': 'show OpenCV debug window; q to exit',
        'filter_straight': 'use calibrated ground-lane tracking and prefer curves at crossings',
        'continuous': 'disable time/distance limits; stop and wait on detection loss',
        'reset_on_exit': 'Gazebo only: save starting world pose and reset on exit; requires --drive',
        'speed': f'requested speed in m/s, 0 < speed <= {MAX_REQUEST_SPEED}',
        'kp': 'pixel-mode steering gain; used without --filter-straight',
        'lookahead': 'forward target distance in metres',
        'lane_width': 'expected ground lane width in metres',
        'max_steer': 'maximum absolute steering angle in radians',
        'seconds': 'time limit; ignored with --continuous',
        'max_distance': 'displacement limit; ignored with --continuous',
        'vehicle_config': 'vehicle YAML; defaults to the installed remote5g_sim configuration',
        'save_dir': 'optional directory for periodic debug screenshots',
    }
    for key, value in asdict(FollowOptions()).items():
        flag = '--'+key.replace('_', '-')
        if isinstance(value, bool):
            parser.add_argument(flag, action=argparse.BooleanOptionalAction, help=descriptions.get(key))
        else:
            parser.add_argument(flag, type=type(value), help=descriptions.get(key))
    parser.set_defaults(**defaults)
    namespace = vars(parser.parse_args(argv))
    config_path = namespace.pop('config')
    if config_path and namespace['vehicle_config']:
        path = Path(namespace['vehicle_config']).expanduser()
        if not path.is_absolute():
            base = Path.cwd() if config.vehicle_config is not None else Path(config_path).expanduser().resolve().parent
            namespace['vehicle_config'] = str(base/path)
    try:
        return FollowOptions(**namespace).validate()
    except ValueError as exc:
        parser.error(str(exc))

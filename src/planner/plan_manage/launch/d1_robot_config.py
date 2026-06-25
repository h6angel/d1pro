"""Load shared D1 robot deployment config (topics + physical limits)."""
import os

import yaml

_CACHE = None


def config_path():
    """Resolve d1_robot.yaml from install share or source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = os.path.join(
            get_package_share_directory('ego_planner'), 'config', 'd1_robot.yaml')
        if os.path.isfile(installed):
            return installed
    except Exception:
        pass

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(launch_dir, '..', 'config', 'd1_robot.yaml'))


def load_d1_robot_config():
    global _CACHE
    if _CACHE is not None:
        return _CACHE
    path = config_path()
    if not os.path.isfile(path):
        raise FileNotFoundError(f'D1 robot config not found: {path}')
    with open(path, 'r', encoding='utf-8') as f:
        _CACHE = yaml.safe_load(f)
    return _CACHE


def nested_get(cfg, dotted_key, fallback=None):
    cur = cfg
    for part in dotted_key.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return fallback
        cur = cur[part]
    return cur

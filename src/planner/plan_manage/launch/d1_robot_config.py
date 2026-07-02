"""Load shared D1 robot deployment config (topics + physical limits + planner tuning)."""
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


def flatten_ros_params(section, prefix):
    """Map yaml section to ROS param keys: section/key -> prefix/key."""
    if not section:
        return {}
    return {f'{prefix}/{k}': v for k, v in section.items()}


def build_ego_planner_params(cfg, overrides=None):
    """
    Build ego_planner_node parameters from d1_robot.yaml.

    overrides: optional dict of ROS param keys (e.g. 'fsm/planning_horizon') to
    replace after assembly — used by launch for CLI/runtime overrides.
    """
    overrides = overrides or {}
    limits = dict(cfg.get('limits', {}))
    camera = cfg.get('camera', {})
    topics = cfg.get('topics', {})
    planner = cfg.get('planner', {})

    params = {}
    params.update(flatten_ros_params(cfg.get('fsm', {}), 'fsm'))
    params.update(flatten_ros_params(cfg.get('grid_map', {}), 'grid_map'))
    params.update(flatten_ros_params(cfg.get('manager', {}), 'manager'))
    params.update(flatten_ros_params(cfg.get('optimization', {}), 'optimization'))
    params.update(flatten_ros_params(cfg.get('bspline', {}), 'bspline'))

    # planner.* -> multiple ROS namespaces (canonical tuning in planner section)
    params['fsm/thresh_replan_time'] = planner['thresh_replan_time']
    params['fsm/collision_check_step'] = planner['collision_check_step']
    params['fsm/thresh_goal_reach_meter'] = planner['goal_reach_thresh']
    params['fsm/planning_horizon'] = planner['planning_horizon']
    params['fsm/tag_stop_dist'] = planner['tag_stop_dist']

    params['grid_map/map_size_x'] = planner['map_size_x']
    params['grid_map/map_size_y'] = planner['map_size_y']
    params['grid_map/map_size_z'] = planner['map_size_z']
    params['grid_map/obstacles_inflation'] = planner['obstacles_inflation']
    params['grid_map/cx'] = camera['cx']
    params['grid_map/cy'] = camera['cy']
    params['grid_map/fx'] = camera['fx']
    params['grid_map/fy'] = camera['fy']
    params['grid_map/frame_id'] = topics['map_frame_id']

    params['manager/planning_horizon'] = planner['planning_horizon']
    params['manager/max_vel'] = limits['max_vel']
    params['manager/max_acc'] = limits['max_acc']

    params['optimization/lambda_fitness'] = planner['lambda_fitness']
    params['optimization/dist0'] = planner['optimization_dist0']
    params['optimization/max_vel'] = limits['max_vel']
    params['optimization/max_acc'] = limits['max_acc']

    params['bspline/limit_vel'] = limits['max_vel']
    params['bspline/limit_acc'] = limits['max_acc']

    for key, value in overrides.items():
        params[key] = value

    return params


def build_traj_server_params(cfg, overrides=None):
    """Build traj_server parameters from d1_robot.yaml."""
    overrides = overrides or {}
    planner = cfg.get('planner', {})
    limits = cfg.get('limits', {})

    params = flatten_ros_params(cfg.get('traj_server', {}), 'traj_server')
    params['traj_server/endpoint_stop_dist'] = planner['goal_reach_thresh']
    params['traj_server/max_yaw_dot'] = limits['max_wz']

    for key, value in overrides.items():
        params[key] = value

    return params

# Copyright (c) EGO Planner contributors.
# Re-exec ros2 launch under tee when save_log is enabled (default: true).

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from datetime import datetime

_DEFAULT_SAVE_LOG = True
_EGO_LOG_FOLDER = 'ego_log'


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def find_workspace_root(start_dir: str) -> str:
    """Walk up to colcon workspace root (has src/ and install/ or .git)."""
    d = os.path.abspath(start_dir)
    for _ in range(16):
        has_src = os.path.isdir(os.path.join(d, 'src'))
        has_install = os.path.isdir(os.path.join(d, 'install'))
        has_git = os.path.isdir(os.path.join(d, '.git'))
        if has_src and (has_install or has_git):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.abspath(start_dir)


def default_ego_log_dir(launch_file: str) -> str:
    """<workspace>/ego_log — project-local, not ~/.ros/log or ~/ego_planner_logs."""
    root = find_workspace_root(os.path.dirname(os.path.abspath(launch_file)))
    return os.path.join(root, _EGO_LOG_FOLDER)


def _filter_passthrough_args(argv: list[str]) -> list[str]:
    """Only user launch overrides (name:=value); never ros2 CLI tokens like 'launch' or package names."""
    passthrough: list[str] = []
    for arg in argv:
        if ':=' in arg:
            name = arg.split(':=', 1)[0]
            if name in ('save_log', 'log_dir'):
                continue
            passthrough.append(arg)
        elif arg in ('--show-args', '--show-arguments'):
            passthrough.append(arg)
    return passthrough


def _strip_launch_log_args(
    argv: list[str], default_log_dir: str, default_save_log: bool = _DEFAULT_SAVE_LOG,
) -> tuple[bool, str, list[str]]:
    save_log = default_save_log
    log_dir = default_log_dir

    for arg in argv:
        if arg.startswith('save_log:='):
            save_log = _parse_bool(arg.split(':=', 1)[1])
        elif arg.startswith('log_dir:='):
            log_dir = os.path.abspath(os.path.expanduser(arg.split(':=', 1)[1]))

    passthrough = _filter_passthrough_args(argv)
    return save_log, log_dir, passthrough


def maybe_reexec_with_log(
    package_name: str,
    launch_file: str,
    default_log_dir: str,
    log_stem: str | None = None,
) -> None:
    """
    If save_log is enabled (default true), re-run this launch and tee output to a file.

    launch_file: stem of the .launch.py file (e.g. d1_planner_bridge).
    log_stem: optional shorter name for the log file only (e.g. d1_bridge).

    Call at module import time (before generate_launch_description returns).
    Inner run sets _EGO_LAUNCH_LOG_ACTIVE=1 to avoid recursion.
    """
    if os.environ.get('_EGO_LAUNCH_LOG_ACTIVE') == '1':
        return

    save_log, log_dir, passthrough = _strip_launch_log_args(
        sys.argv[1:], default_log_dir, _DEFAULT_SAVE_LOG)
    if not save_log:
        return

    file_stem = log_stem or launch_file
    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(log_dir, f'{package_name}_{file_stem}_{stamp}.log')

    inner_cmd = ['ros2', 'launch', package_name, f'{launch_file}.launch.py'] + passthrough
    header = (
        f'===== {datetime.now().isoformat()} =====\n'
        f'package: {package_name}\n'
        f'launch: {launch_file}.launch.py\n'
        f'command: {shlex.join(inner_cmd)}\n'
        f'log_dir: {log_dir}\n'
        '===== output =====\n'
    )

    with open(log_path, 'w', encoding='utf-8') as f:
        f.write(header)

    print(f'[launch_log] Logging to: {log_path}', flush=True)

    env = os.environ.copy()
    env['_EGO_LAUNCH_LOG_ACTIVE'] = '1'

    proc = subprocess.Popen(
        inner_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
        bufsize=1,
        text=True,
        errors='replace',
    )
    assert proc.stdout is not None
    with open(log_path, 'a', encoding='utf-8', buffering=1) as log_f:
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            log_f.write(line)
    sys.exit(proc.wait())

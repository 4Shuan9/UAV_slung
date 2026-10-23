#!/usr/bin/env python3
"""Launch the UAV slung-load Gazebo/PX4 simulation stack.

This launch file deliberately keeps project simulation assets outside PX4.
Custom worlds/models are loaded from <project_root>/sim while PX4's stock
models are still available through GZ_SIM_RESOURCE_PATH.
"""

import os
from pathlib import Path
import shutil
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _cleanup_processes(context=None, *args, **kwargs):
    if context is not None:
        try:
            enabled = _as_bool(LaunchConfiguration("cleanup").perform(context))
        except Exception:
            enabled = True
        if not enabled:
            return []

    print("[uav_slung_sim] Cleaning stale PX4 / Gazebo / MicroXRCEAgent processes ...")
    term_cmds = [
        ["pkill", "-TERM", "-x", "px4"],
        ["pkill", "-TERM", "-x", "MicroXRCEAgent"],
        ["pkill", "-TERM", "-f", "gz sim"],
    ]
    kill_cmds = [
        ["pkill", "-KILL", "-x", "px4"],
        ["pkill", "-KILL", "-x", "MicroXRCEAgent"],
        ["pkill", "-KILL", "-f", "gz sim"],
    ]

    for cmd in term_cmds:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    time.sleep(0.6)
    for cmd in kill_cmds:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    return []


def _build_runtime_actions(context, *args, **kwargs):
    project_root = Path(LaunchConfiguration("project_root").perform(context)).expanduser().resolve()
    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser().resolve()
    world_arg = LaunchConfiguration("world").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    agent_port = LaunchConfiguration("agent_port").perform(context)
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    runtime_dir = Path(LaunchConfiguration("runtime_dir").perform(context)).expanduser().resolve()
    if runtime_dir == project_root or project_root in runtime_dir.parents:
        raise RuntimeError("runtime_dir must be outside the project directory")
    runtime_dir.mkdir(parents=True, exist_ok=True)

    world_candidate = Path(world_arg).expanduser()
    if world_candidate.is_absolute():
        world_file = world_candidate
    else:
        world_file = project_root / "sim" / "worlds" / world_arg

    sim_models = project_root / "sim" / "models"
    px4_models = px4_dir / "Tools" / "simulation" / "gz" / "models"
    px4_bin = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"

    if not world_file.is_file():
        raise RuntimeError(f"World file not found: {world_file}")
    if not sim_models.is_dir():
        raise RuntimeError(f"Simulation models directory not found: {sim_models}")
    if not px4_bin.is_file():
        raise RuntimeError(
            f"PX4 SITL binary not found: {px4_bin}\n"
            "Build PX4 SITL first, or pass px4_dir:=/path/to/PX4-Autopilot"
        )
    if shutil.which("MicroXRCEAgent") is None:
        raise RuntimeError("MicroXRCEAgent was not found in PATH")
    if shutil.which("gz") is None:
        raise RuntimeError("Gazebo 'gz' executable was not found in PATH")

    resource_paths = [str(sim_models)]
    if px4_models.is_dir():
        resource_paths.append(str(px4_models))
    inherited = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if inherited:
        resource_paths.append(inherited)
    resource_path = os.pathsep.join(resource_paths)

    # x500_mono_cam_down is expected from the configured PX4 environment or an
    # existing Gazebo resource path. Warn rather than fail because it may also
    # be provided by another resource directory / local Gazebo cache.
    model_local = sim_models / model_name
    model_px4 = px4_models / model_name
    warning_actions = []
    if not model_local.exists() and not model_px4.exists():
        warning_actions.append(
            LogInfo(
                msg=(
                    f"[uav_slung_sim] WARNING: model '{model_name}' was not found under "
                    f"{sim_models} or {px4_models}. It must exist in another Gazebo resource "
                    "path (the original project used a PX4 environment providing this model)."
                )
            )
        )

    plugin_dir = str(Path(get_package_prefix("slung_ctrl")) / "lib")
    inherited_plugins = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    plugin_path = plugin_dir + (os.pathsep + inherited_plugins if inherited_plugins else "")
    common_env = {"GZ_SIM_RESOURCE_PATH": resource_path,
                  "GZ_SIM_SYSTEM_PLUGIN_PATH": plugin_path}

    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
        output="screen",
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r"] + (["-s", "--headless-rendering"] if headless else []) + [str(world_file)],
        additional_env=common_env,
        output="screen",
    )

    px4_env = dict(common_env)
    px4_env["PX4_GZ_MAGNETOMETER"] = "enu_tesla"
    px4_env["PX4_GZ_MODEL_NAME"] = model_name
    px4_env["PX4_UXRCE_DDS_PORT"] = agent_port
    px4_env["PX4_GZ_WORLD"] = world_file.stem
    px4 = ExecuteProcess(
        cmd=[str(px4_bin), "-d", str(px4_bin.parent.parent / "etc"), "-w", str(runtime_dir)],
        cwd=str(runtime_dir),
        additional_env=px4_env,
        output="screen",
    )

    vision = Node(
        package="slung_ctrl",
        executable="payload_detector.py",
        name="payload_detector",
        additional_env={"SLUNG_SHOW_IMAGE": "0" if headless else "1"},
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_vision")),
    )

    return [
        LogInfo(msg=f"[uav_slung_sim] project_root: {project_root}"),
        LogInfo(msg=f"[uav_slung_sim] world       : {world_file}"),
        LogInfo(msg=f"[uav_slung_sim] PX4         : {px4_dir}"),
        LogInfo(msg=f"[uav_slung_sim] model       : {model_name}"),
        LogInfo(msg=f"[uav_slung_sim] runtime     : {runtime_dir}"),
        *warning_actions,
        # Start in stages to avoid Gazebo/PX4 startup races.
        TimerAction(period=0.5, actions=[agent]),
        TimerAction(period=1.0, actions=[gazebo]),
        TimerAction(period=3.0, actions=[px4]),
        TimerAction(period=5.0, actions=[vision]),
    ]


def generate_launch_description():
    default_root = os.environ.get("UAV_SLUNG_ROOT", os.path.expanduser("~/UAV_slung"))

    return LaunchDescription([
        SetEnvironmentVariable('PX4_SYS_AUTOSTART', '4014'),
        SetEnvironmentVariable('PX4_SIM_MODEL', 'gz_x500_mono_cam_down'),
        SetEnvironmentVariable('PX4_GZ_STANDALONE', '1'),
        SetEnvironmentVariable('PX4_PARAM_UXRCE_DDS_SYNCT', '0'),
        SetEnvironmentVariable('PX4_PARAM_UXRCE_DDS_PTCFG',
                               EnvironmentVariable('ROS_LOCALHOST_ONLY', default_value='0')),
        DeclareLaunchArgument(
            "project_root",
            default_value=default_root,
            description="UAV_slung project root containing sim/ and ros2_ws/",
        ),
        DeclareLaunchArgument(
            "px4_dir",
            default_value=os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot")),
            description="PX4-Autopilot source/build directory",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="UAV_slung_world.sdf",
            description="World filename under <project_root>/sim/worlds, or an absolute path",
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="x500_mono_cam_down",
            description="Gazebo model name used by PX4 SITL",
        ),
        DeclareLaunchArgument(
            "agent_port",
            default_value="8888",
            description="Micro XRCE-DDS Agent UDP port",
        ),
        DeclareLaunchArgument(
            "start_vision",
            default_value="true",
            description="Start payload_detector.py together with the simulator",
        ),
        DeclareLaunchArgument("headless", default_value="false", description="Run without Gazebo/OpenCV windows"),
        DeclareLaunchArgument("runtime_dir", default_value=os.path.join(os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state")), "UAV_slung", "px4"),
                              description="PX4 parameters and logs; must be outside the project directory"),
        DeclareLaunchArgument(
            "cleanup",
            default_value="false",
            description="Clean stale px4 / gz sim / MicroXRCEAgent processes before and after launch",
        ),
        OpaqueFunction(function=_cleanup_processes),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_processes)])
        ),
        OpaqueFunction(function=_build_runtime_actions),
    ])

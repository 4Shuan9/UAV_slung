#!/usr/bin/env python3
"""Launch the UAV slung-load Gazebo/PX4 simulation stack.

This launch file deliberately keeps project simulation assets outside PX4.
Custom worlds/models are loaded from <project_root>/sim while PX4's stock
models are still available through GZ_SIM_RESOURCE_PATH.
"""

import os
import math
from pathlib import Path
import shutil
import subprocess
import time
import tempfile
import xml.etree.ElementTree as ET

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


# Codex-generated: bounded, purposeful football-field activities.
def _configure_people(actors):

    def capsule(x1, x2, y, r, n=24):
        # A straight shuttle with semicircular turns, avoiding abrupt reversals.
        p = [(x1, y-r), (x2, y-r)]
        p.extend((x2 + r*math.cos(-math.pi/2 + math.pi*i/n),
                  y + r*math.sin(-math.pi/2 + math.pi*i/n)) for i in range(1, n+1))
        p.append((x1, y+r))
        p.extend((x1 + r*math.cos(math.pi/2 + math.pi*i/n),
                  y + r*math.sin(math.pi/2 + math.pi*i/n)) for i in range(1, n+1))
        return p

    def rounded_rect(xmin, xmax, ymin, ymax, radius, n=16):
        points = []
        for cx, cy, start in [(xmax-radius, ymin+radius, -math.pi/2),
                              (xmax-radius, ymax-radius, 0),
                              (xmin+radius, ymax-radius, math.pi/2),
                              (xmin+radius, ymin+radius, math.pi)]:
            points.extend((cx + radius*math.cos(start + i*math.pi/(2*n)),
                           cy + radius*math.sin(start + i*math.pi/(2*n)))
                          for i in range(n+1))
        return points + [points[0]]

    routes = {
        # World coordinates: the field is rotated 90 degrees and centred at (54, 0).
        'sideline_1': (rounded_rect(19.0, 89.0, -55.5, 55.5, 2.0), 1.75),
        'sideline_2': ([(20.10 + y, x) for x, y in capsule(-20.0, 20.0, 0.0, 1.0)], 1.75),
    }
    for actor in actors:
        name = actor.get('name')
        trajectory = actor.find('script/trajectory')
        if trajectory is None:
            raise RuntimeError(f'Missing trajectory for {name}')
        def add_animation(key, filename):
            anim = ET.Element('animation', name=key)
            actor.insert(list(actor).index(actor.find('script')), anim)
            ET.SubElement(anim, 'filename').text = 'model://football_player/meshes/' + filename
            ET.SubElement(anim, 'scale').text = '1.0'
            ET.SubElement(anim, 'interpolate_x').text = 'false'

        script = actor.find('script')
        if name in ('sideline_1', 'sideline_2'):
            # Shared in-place walking animation, with speed matched to its stride.
            actor.find("animation[@name='move']/filename").text = (
                'model://football_player/meshes/players_navy10/walk.dae')
        if name in ('runner_1', 'runner_2', 'runner_3'):
            for old_animation in actor.findall('animation'):
                actor.remove(old_animation)
            for tr in script.findall('trajectory'):
                script.remove(tr)
            add_animation('squat', 'players_navy10/squat.dae')
            add_animation('stretch', 'players_navy10/stretch.dae')
            if name == 'runner_3':
                add_animation('goal_b', 'players_white9/goal_idle_b.dae')
                parts = [('goal_b', 4.633333), ('squat', 3.733333),
                         ('goal_b', 4.633333), ('stretch', 8.866667)]
                for idx, (kind, duration) in enumerate(parts):
                    tr = ET.SubElement(script, 'trajectory', id=str(idx), type=kind)
                    for t in (0.0, duration):
                        w = ET.SubElement(tr, 'waypoint')
                        ET.SubElement(w, 'time').text = str(t)
                        ET.SubElement(w, 'pose').text = '54.0 50.5 0.17 -1.570796 0 0'
            else:
                add_animation('run', 'players_navy10/run.dae' if name == 'runner_1' else 'players_navy10/run_slow.dae')
                run_speed, cycle = ((4.86, 0.7) if name == 'runner_1' else (4.05, 0.84))
                # Different lanes, direction, length and warm-up order.
                route = capsule(26.0, 82.0, 18.0 if name == 'runner_1' else -18.0,
                                3.0 if name == 'runner_1' else 4.0)
                points = route if name == 'runner_1' else list(reversed(route))
                split = len(points)//2
                halves = [points[:split+1], points[split:]]
                for j, path in enumerate(halves):
                    # Densify straight segments, then ramp speed near the stops.
                    dense = [path[0]]
                    for first, second in zip(path, path[1:]):
                        count = max(1, math.ceil(math.dist(first, second) / 0.75))
                        dense.extend((first[0] + (second[0] - first[0])*k/count,
                                      first[1] + (second[1] - first[1])*k/count)
                                     for k in range(1, count+1))
                    path = dense
                    distances = [0.0]
                    for first, second in zip(path, path[1:]):
                        distances.append(distances[-1] + math.dist(first, second))
                    length = distances[-1]
                    speeds = [min(run_speed, math.sqrt(2 * 2.5 * min(d, length-d)))
                              for d in distances]
                    tr = ET.SubElement(script, 'trajectory', id=str(2*j), type='run', tension='1.0')
                    elapsed, prev_yaw = 0.0, None
                    for i, point in enumerate(path):
                        if i:
                            elapsed += 2 * math.dist(path[i-1], point) / (speeds[i-1] + speeds[i])
                        before = path[max(i-1,0)]
                        after = path[min(i+1,len(path)-1)]
                        yaw = math.atan2(after[1]-before[1], after[0]-before[0]) + math.pi/2
                        if prev_yaw is not None:
                            yaw = prev_yaw + (yaw-prev_yaw+math.pi)%math.tau-math.pi
                        prev_yaw = yaw
                        w = ET.SubElement(tr, 'waypoint')
                        ET.SubElement(w, 'time').text = f'{elapsed:.5f}'
                        ET.SubElement(w, 'pose').text = f'{point[0]:.5f} {point[1]:.5f} 0.17 -1.570796 0 {yaw:.6f}'
                    # End the run at a full animation cycle, ready for blending.
                    scale = math.ceil(elapsed / cycle) * cycle / elapsed
                    for w in tr.findall('waypoint'):
                        w.find('time').text = f"{float(w.findtext('time')) * scale:.5f}"
                    kind, duration = ((('squat', 3.733333), ('stretch', 8.866667))[j]
                                      if name == 'runner_1' else
                                      (('stretch', 8.866667), ('squat', 3.733333))[j])
                    pose = f'{path[-1][0]:.5f} {path[-1][1]:.5f} 0.17 -1.570796 0 {prev_yaw:.6f}'
                    hold = ET.SubElement(script, 'trajectory', type=kind)
                    for t in (0.0, duration):
                        w = ET.SubElement(hold, 'waypoint')
                        ET.SubElement(w, 'time').text = str(t)
                        ET.SubElement(w, 'pose').text = pose
            for idx, tr in enumerate(script.findall('trajectory')):
                tr.set('id', str(idx))
            continue
        if name not in routes:
            raise RuntimeError(f'Unexpected people actor: {name}')
        points, speed = routes[name]
        old = trajectory.findall('waypoint')
        if not old:
            raise RuntimeError(f'Missing waypoint for {name}')
        base = list(map(float, old[0].findtext('pose').split()))
        for waypoint in old:
            trajectory.remove(waypoint)
        elapsed, previous_yaw = 0.0, None
        for i, point in enumerate(points):
            if i:
                elapsed += math.dist(points[i-1], point) / speed
            before = points[i-1] if i else points[-2]
            after = points[i+1] if i < len(points)-1 else points[1]
            yaw = math.atan2(after[1]-before[1], after[0]-before[0]) + math.pi/2
            if previous_yaw is not None:
                yaw = previous_yaw + (yaw-previous_yaw+math.pi) % math.tau-math.pi
            previous_yaw = yaw
            w = ET.SubElement(trajectory, 'waypoint')
            ET.SubElement(w, 'time').text = f'{elapsed:.5f}'
            ET.SubElement(w, 'pose').text = (
                f'{point[0]:.5f} {point[1]:.5f} {base[2] - 0.11:.6f} '
                f'{base[3]:.6f} {base[4]:.6f} {yaw:.6f}')


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

    # Codex-generated: optional animated crowd; baseline world stays byte-identical.
    # Keep the runtime filename and SDF world name so PX4 and camera topics match.
    source_world_file = world_file
    people_enabled = _as_bool(LaunchConfiguration("people").perform(context))
    temp_people_world = None
    if people_enabled:
        people_file = project_root / "sim" / "worlds" / "football_player.sdf"
        if not people_file.is_file():
            raise RuntimeError(f"People scene not found: {people_file}")
        actor_world = ET.parse(people_file).getroot().find("world")
        if actor_world is None:
            raise RuntimeError(f"People scene has no world: {people_file}")
        actors = actor_world.findall("actor")
        if not actors:
            raise RuntimeError(f"People scene has no actors: {people_file}")
        _configure_people(actors)
        world_text = source_world_file.read_text(encoding="utf-8")
        if world_text.count("</world>") != 1:
            raise RuntimeError("Expected exactly one closing world tag")
        temp_people_world = tempfile.TemporaryDirectory(prefix="slung_people_", dir=runtime_dir)
        world_file = Path(temp_people_world.name) / source_world_file.name
        fragment = "\n".join(ET.tostring(a, encoding="unicode") for a in actors)
        world_file.write_text(world_text.replace("</world>", fragment + "\n  </world>", 1), encoding="utf-8")

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

    # Codex-generated: load a scene-specific GUI config, including its window style.
    gui_config = source_world_file.parent / "gui.config"
    gui_args = ["--gui-config", str(gui_config)] if not headless and gui_config.is_file() else []
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r"] + (["-s", "--headless-rendering"] if headless else []) + gui_args + [str(world_file)],
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

    def cleanup_people(_event, _context):
        if temp_people_world is not None:
            temp_people_world.cleanup()

    cleanup_people_action = RegisterEventHandler(OnShutdown(on_shutdown=cleanup_people))
    return [
        cleanup_people_action,
        LogInfo(msg=f"[uav_slung_sim] project_root: {project_root}"),
        LogInfo(msg=f"[uav_slung_sim] world       : {world_file}"),
        LogInfo(msg=f"[uav_slung_sim] people      : {people_enabled}"),
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
        DeclareLaunchArgument("people", default_value="true", choices=["true", "false"], description="Show optional animated walkers and runners"),
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

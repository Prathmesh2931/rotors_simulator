"""
swarm.launch.py
───────────────
Dynamic multi-robot launch for swarm_bringup.

HOW IT WORKS
────────────
1.  Reads  config/robots.yaml  (the ONLY file you ever edit to add/remove robots)
2.  Auto-generates  config/swarm_bridge_config.yaml  at launch time via
    generate_bridge_config.py  → no more hand-editing bridge entries
3.  Starts Gazebo with the project world
4.  Starts ONE shared ros_gz_bridge node covering all robots
5.  For every robot in robots.yaml:
        a) Spawns the SDF model at the configured pose (staggered timers)
        b) Starts a namespaced controller node
        c) Starts a namespaced interface node

TO ADD A ROBOT
──────────────
Open  config/robots.yaml, add one entry under `robots:`.  Done.
No changes needed here, in the bridge YAML, or anywhere else.

TO CHANGE A TOPIC TYPE
──────────────────────
Edit  bridge_topics  in  config/robots.yaml.  Done.
"""

import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _pkg(name: str) -> str:
    """Return the share directory for a ROS 2 package."""
    return get_package_share_directory(name)


def _load_fleet(robots_yaml: str) -> dict:
    """Parse robots.yaml and return the fleet dict."""
    with open(robots_yaml) as f:
        return yaml.safe_load(f)


def _generate_bridge_inline(robots_yaml: str, bridge_yaml: str) -> None:
    import yaml as _yaml
    with open(robots_yaml) as f:
        fleet = _yaml.safe_load(f)
    robots        = fleet.get("robots", [])
    bridge_tpls   = fleet.get("bridge_topics", [])
    shared_topics = fleet.get("shared_bridge_topics", [])
    entries = []
    for robot in robots:
        name = robot["name"]
        for tpl in bridge_tpls:
            entries.append({
                "ros_topic_name": tpl["ros_template"].replace("{name}", name),
                "gz_topic_name":  tpl["gz_template"].replace("{name}", name),
                "ros_type_name":  tpl["ros_type"],
                "gz_type_name":   tpl["gz_type"],
                "direction":      tpl["direction"],
            })
    for topic in shared_topics:
        entries.append({
            "ros_topic_name": topic["ros_topic_name"],
            "gz_topic_name":  topic["gz_topic_name"],
            "ros_type_name":  topic["ros_type_name"],
            "gz_type_name":   topic["gz_type_name"],
            "direction":      topic["direction"],
        })
    with open(bridge_yaml, "w") as f:
        f.write("# AUTO-GENERATED at launch time — edit robot.yaml instead\n---\n")
        _yaml.dump(entries, f, default_flow_style=False)
    print(f"\033[36m[bridge_gen] Wrote {len(entries)} entries → {bridge_yaml}\033[0m")

def _generate_robot_sdf(template_path: str, robot_name: str) -> str:
    with open(template_path) as f:
        content = f.read()
    content = content.replace("{robot_name}", robot_name)
    output_path = f"/tmp/{robot_name}_swift_pico.sdf"
    with open(output_path, "w") as f:
        f.write(content)
    print(f"\033[36m[sdf_gen] {robot_name} → {output_path}\033[0m")
    return output_path
# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction — runs at launch time with full Python context
# ─────────────────────────────────────────────────────────────────────────────

def _build_entities(context, *args, **kwargs):
    """
    Reads robots.yaml, regenerates bridge config, then returns the full
    list of launch entities (Gazebo + bridge + per-robot spawn/controller).
    """

    # ── Resolve package paths ─────────────────────────────────────────────
    pkg_bringup     = _pkg("rotors_swift_gazebo")
    pkg_description = _pkg("rotors_swift_description")
    pkg_ros_gz_sim  = _pkg("ros_gz_sim")
    scripts_dir     = os.path.join(pkg_bringup, "src")

    robots_yaml = os.path.join(pkg_bringup, "config", "robots.yaml")
    bridge_yaml = os.path.join(pkg_bringup, "config", "swarm_bridge_config.yaml")
    world_path  = os.path.join(pkg_bringup, "worlds", "swift_pico_world.sdf")
    models_dir  = os.path.join(pkg_description, "models", "swift_pico")

    # ── Step 1: Regenerate bridge config ─────────────────────────────────
    _generate_bridge_inline(robots_yaml, bridge_yaml)

    # ── Step 2: Load fleet ────────────────────────────────────────────────
    fleet  = _load_fleet(robots_yaml)
    robots = fleet.get("robots", [])

    if not robots:
        raise RuntimeError("No robots defined in robots.yaml!")

    print(f"\033[32m[swarm_launch] Launching {len(robots)} robot(s): "
          f"{[r['name'] for r in robots]}\033[0m")

    # ── Step 3: Gazebo ────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    # ── Step 4: Single shared bridge node (auto-generated config) ────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="swarm_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    entities = [
        LogInfo(msg="──────────────────────────────────────────"),
        LogInfo(msg=f"Swarm size : {len(robots)} robots"),
        LogInfo(msg=f"Bridge cfg : {bridge_yaml}  (auto-generated)"),
        LogInfo(msg="──────────────────────────────────────────"),
        gz_sim,
        bridge,
    ]

    # ── Step 5: Per-robot spawn + nodes ───────────────────────────────────
    #   Timers:
    #     t=3s          → spawn robot i=0  (Gazebo needs a few seconds to start)
    #     t=3+i*2s      → spawn subsequent robots
    #     t=spawn+2s    → start controller and interface (model must be loaded first)

    GAZEBO_STARTUP_DELAY = 3.0   # seconds to wait before first spawn
    SPAWN_INTERVAL       = 2.0   # seconds between consecutive spawns
    NODE_START_DELAY     = 2.0   # seconds after spawn before starting nodes

    for i, robot in enumerate(robots):
        name     = robot["name"]
        sdf_file = robot["sdf"]
        x        = str(robot.get("x", 0.0))
        y        = str(robot.get("y", 0.0))
        z        = str(robot.get("z", 0.2))

        spawn_t      = GAZEBO_STARTUP_DELAY + i * SPAWN_INTERVAL
        node_start_t = spawn_t + NODE_START_DELAY

        template_path = os.path.join(models_dir, sdf_file)
        sdf_path = _generate_robot_sdf(template_path, name)

        # -- a) Spawn the SDF model ------------------------------------------
        spawn = TimerAction(
            period=spawn_t,
            actions=[
                LogInfo(msg=f"[{name}] Spawning at ({x}, {y}, {z})  sdf={sdf_file}"),
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-name", name,
                        "-file", sdf_path,
                        "-x", x, "-y", y, "-z", z,
                    ],
                    output="screen",
                ),
            ],
        )

        # -- b) Controller node  (namespace: /<name>/rotors) -----------------
        controller = TimerAction(
            period=node_start_t,
            actions=[
                Node(
                    package="rotors_control",
                    executable="roll_pitch_yawrate_thrust_controller_node",
                    name="controller",
                    namespace=f"{name}/rotors",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
            ],
        )

        # -- c) Interface node  (namespace: /<name>/rotors) ------------------
        interface = TimerAction(
            period=node_start_t,
            actions=[
                Node(
                    package="rotors_swift_interface",
                    executable="rotors_swift_interface",
                    name="interface",
                    namespace=f"{name}/rotors",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
            ],
        )

        entities.extend([spawn, controller, interface])

    return entities


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([
        # Optional: allow overriding the world file from the CLI
        DeclareLaunchArgument(
            "world",
            default_value="",
            description="Path to an alternative .sdf world file (leave blank for default)",
        ),
        OpaqueFunction(function=_build_entities),
    ])
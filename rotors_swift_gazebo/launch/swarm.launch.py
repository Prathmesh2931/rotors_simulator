import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _gen_bridge(robots_yaml, bridge_yaml):
    with open(robots_yaml) as f:
        fleet = yaml.safe_load(f)

    entries = []
    for robot in fleet.get("robots", []):
        name = robot["name"]
        for tpl in fleet.get("bridge_topics", []):
            entries.append({
                "ros_topic_name": tpl["ros_template"].replace("{name}", name),
                "gz_topic_name":  tpl["gz_template"].replace("{name}", name),
                "ros_type_name":  tpl["ros_type"],
                "gz_type_name":   tpl["gz_type"],
                "direction":      tpl["direction"],
            })
    for topic in fleet.get("shared_bridge_topics", []):
        entries.append({
            "ros_topic_name": topic["ros_topic_name"],
            "gz_topic_name":  topic["gz_topic_name"],
            "ros_type_name":  topic["ros_type_name"],
            "gz_type_name":   topic["gz_type_name"],
            "direction":      topic["direction"],
        })

    with open(bridge_yaml, "w") as f:
        f.write("# generated at launch time — edit robots.yaml instead\n---\n")
        yaml.dump(entries, f, default_flow_style=False)

    print(f"[bridge_gen] {len(entries)} entries -> {bridge_yaml}")


def _gen_sdf(template_path, robot_name):
    with open(template_path) as f:
        content = f.read()
    out = f"/tmp/{robot_name}_swift_pico.sdf"
    with open(out, "w") as f:
        f.write(content.replace("{robot_name}", robot_name))
    print(f"[sdf_gen] {robot_name} -> {out}")
    return out


def _build(context, *args, **kwargs):
    bringup     = get_package_share_directory("rotors_swift_gazebo")
    description = get_package_share_directory("rotors_swift_description")
    gz_pkg      = get_package_share_directory("ros_gz_sim")

    robots_yaml = os.path.join(bringup, "config", "robots.yaml")
    bridge_yaml = os.path.join(bringup, "config", "swarm_bridge_config.yaml")
    world       = os.path.join(bringup, "worlds", "swift_pico_world.sdf")
    models_dir  = os.path.join(description, "models", "swift_pico")

    _gen_bridge(robots_yaml, bridge_yaml)

    with open(robots_yaml) as f:
        robots = yaml.safe_load(f).get("robots", [])

    if not robots:
        raise RuntimeError("No robots defined in robots.yaml")

    print(f"[swarm] launching {len(robots)} robot(s): {[r['name'] for r in robots]}")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="swarm_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    entities = [
        LogInfo(msg=f"swarm size: {len(robots)} | bridge: {bridge_yaml}"),
        gz,
        bridge,
    ]

    # stagger spawns: first robot at t=3s, then every 2s
    # controller/interface start 2s after their robot spawns
    for i, robot in enumerate(robots):
        name  = robot["name"]
        x, y, z = str(robot.get("x", 0.0)), str(robot.get("y", 0.0)), str(robot.get("z", 0.2))

        sdf_path   = _gen_sdf(os.path.join(models_dir, robot["sdf"]), name)
        t_spawn    = 3.0 + i * 2.0
        t_nodes    = t_spawn + 2.0

        entities.append(TimerAction(period=t_spawn, actions=[
            LogInfo(msg=f"[{name}] spawning at ({x}, {y}, {z})"),
            Node(package="ros_gz_sim", executable="create",
                 arguments=["-name", name, "-file", sdf_path, "-x", x, "-y", y, "-z", z],
                 output="screen"),
        ]))

        entities.append(TimerAction(period=t_nodes, actions=[
            Node(package="rotors_control",
                 executable="roll_pitch_yawrate_thrust_controller_node",
                 name="controller", namespace=f"{name}/rotors",
                 parameters=[{"use_sim_time": True}], output="screen"),
        ]))

        entities.append(TimerAction(period=t_nodes, actions=[
            Node(package="rotors_swift_interface",
                 executable="rotors_swift_interface",
                 name="interface", namespace=f"{name}/rotors",
                 parameters=[{"use_sim_time": True}], output="screen"),
        ]))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="",
                              description="override world sdf path"),
        OpaqueFunction(function=_build),
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # 1. Get Package Paths
    pkg_project_bringup = get_package_share_directory('rotors_swift_gazebo')
    pkg_project_description = get_package_share_directory('rotors_swift_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Define the Full Path to World File (THIS WAS MISSING)
    world_path = os.path.join(pkg_project_bringup, 'worlds', 'swift_pico_world.sdf')

    # 3. Launch the Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(), 
    )

    # 4. Swarm Config
    swarm_config = [
        ('master', 0.0, 0.0),
        ('slave1', 1.0, 1.0),
        ('slave2', -1.0, 1.0),
        ('slave3', 0.0, -1.0)
    ]

    launch_entities = [gz_sim]

    for robot_name, x_pos, y_pos in swarm_config:
        
        bridge_args = [
            f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            f'/model/{robot_name}/rotors/command/motor_speed@actuator_msgs/msg/Actuators@ignition.msgs.Actuators',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ]

        drone_group = GroupAction([
            PushRosNamespace(robot_name),

            # Spawn Robot
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', robot_name, 
                            '-file', os.path.join(pkg_project_description, 'models', 'swift_pico', 'swift_pico.sdf'),
                            '-x', str(x_pos), '-y', str(y_pos), '-z', '0.2'],
                 output='screen'),

            # Controller (Default Params)
            Node(package='rotors_control', namespace='rotors',
                 executable='roll_pitch_yawrate_thrust_controller_node',
                 name='controller',
                 output='screen',
                 parameters=[{'use_sim_time': True}]),

            # Interface
            Node(package='rotors_swift_interface', namespace='rotors',
                 executable='rotors_swift_interface',
                 name='interface',
                 output='screen',
                 parameters=[{'use_sim_time': True}]),

            # Bridge
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=bridge_args,
                 output='screen',
                 remappings=[
                     (f'/model/{robot_name}/odometry', f'rotors/odometry'),
                     (f'/model/{robot_name}/rotors/command/motor_speed', f'rotors/command/motor_speed')
                 ])
        ])
        launch_entities.append(drone_group)

    return LaunchDescription(launch_entities)
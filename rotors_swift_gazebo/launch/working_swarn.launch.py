import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    pkg_project_bringup = get_package_share_directory('rotors_swift_gazebo')
    pkg_project_description = get_package_share_directory('rotors_swift_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file
    world_path = os.path. join(pkg_project_bringup, 'worlds', 'swift_pico_world.sdf')

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path. join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(), 
    )

    # Bridge with fixed config path
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'swarm_bridge_config.yaml'),
        }],
        output='screen'
    )

    # Swarm config
    swarm_config = [
        ('master', 'master_swift_pico.sdf', 0.0, 0.0),
        ('slave1', 'slave1_swift_pico.sdf', 1.0, 1.0),
        ('slave2', 'slave2_swift_pico.sdf', -1.0, 1.0),
        ('slave3', 'slave3_swift_pico.sdf', 0.0, -1.0)
    ]

    launch_entities = [gz_sim, bridge]

    for i, (robot_name, sdf_file, x_pos, y_pos) in enumerate(swarm_config):
        
        # Spawn Robot
        spawn_robot = TimerAction(
            period=3.0 + i * 2.0,
            actions=[
                Node(package='ros_gz_sim', executable='create',
                     arguments=['-name', robot_name, 
                                '-file', os.path.join(pkg_project_description, 'models', 'swift_pico', sdf_file),
                                '-x', str(x_pos), '-y', str(y_pos), '-z', '0.2'],
                     output='screen')
            ]
        )
        
        # Controller with EXPLICIT namespace 
        controller = TimerAction(
            period=5.0 + i * 2.0,
            actions=[
                Node(package='rotors_control',
                     executable='roll_pitch_yawrate_thrust_controller_node',
                     name='controller',
                     namespace=f'{robot_name}/rotors',  # EXPLICIT: /master/rotors/, /slave1/rotors/, etc. 
                     output='screen',
                     parameters=[{'use_sim_time': True}])
            ]
        )

        # Interface with EXPLICIT namespace  
        interface = TimerAction(
            period=5.0 + i * 2.0,
            actions=[
                Node(package='rotors_swift_interface',
                     executable='rotors_swift_interface',
                     name='interface',
                     namespace=f'{robot_name}/rotors',  # EXPLICIT: /master/rotors/, /slave1/rotors/, etc. 
                     output='screen',
                     parameters=[{'use_sim_time': True}])
            ]
        )
        
        launch_entities.extend([spawn_robot, controller, interface])

    return LaunchDescription(launch_entities)
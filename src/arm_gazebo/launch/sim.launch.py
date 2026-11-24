import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('arm_description')
    gz_share = get_package_share_directory('arm_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(desc_share, 'urdf', 'robotic_arm.urdf')
    rviz_config = os.path.join(desc_share, 'rviz', 'arm.rviz')
    worlds_dir = os.path.join(gz_share, 'worlds')
    bridge_config = os.path.join(gz_share, 'config', 'gz_bridge.yaml')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    use_rviz = LaunchConfiguration('use_rviz')

    # Add worlds directory to GZ_SIM_RESOURCE_PATH so spaces in workspace paths work cleanly
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f'{worlds_dir}:{existing_gz_path}' if existing_gz_path else worlds_dir
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r arm_world.sdf'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robotic_arm',
        output='screen',
        arguments=[
            '-name', 'robotic_arm',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    gz_trajectory_adapter = Node(
        package='arm_controller',
        executable='gz_trajectory_adapter',
        name='gz_trajectory_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 alongside Gazebo',
        ),
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_arm,
        gz_bridge,
        gz_trajectory_adapter,
        rviz2,
    ])

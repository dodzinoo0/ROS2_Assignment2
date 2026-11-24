import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_share = get_package_share_directory('arm_gazebo')

    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    dry_run = LaunchConfiguration('dry_run')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    hardware_bridge = Node(
        package='arm_hardware_bridge',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baudrate': baudrate,
            'dry_run': dry_run,
        }],
    )

    arm_gui = Node(
        package='arm_controller',
        executable='arm_gui',
        name='arm_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Arduino USB serial port (e.g. /dev/ttyACM0 or /dev/ttyUSB0)',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baud rate matching the Arduino sketch',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, log serial frames without opening a physical serial port',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch the Tkinter slider GUI for manual joint control',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 alongside Gazebo',
        ),
        sim_launch,
        hardware_bridge,
        arm_gui,
    ])

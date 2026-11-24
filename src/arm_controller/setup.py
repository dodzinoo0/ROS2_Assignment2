from setuptools import find_packages, setup

package_name = 'arm_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='JointTrajectory publisher, Gazebo trajectory adapter, and slider GUI for the robotic arm.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gz_trajectory_adapter = arm_controller.gz_trajectory_adapter:main',
            'trajectory_publisher = arm_controller.trajectory_publisher:main',
            'arm_gui = arm_controller.arm_gui:main',
        ],
    },
)

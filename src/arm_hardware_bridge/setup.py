from setuptools import find_packages, setup

package_name = 'arm_hardware_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS 2 USB serial hardware bridge for Arduino servo control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = arm_hardware_bridge.serial_bridge_node:main',
        ],
    },
)

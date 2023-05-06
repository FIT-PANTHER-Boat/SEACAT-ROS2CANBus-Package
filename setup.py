from setuptools import setup

package_name = 'ros2_can_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Braidan',
    maintainer_email='bduffy2018@my.fit.edu',
    description='A package that contains nodes that enable ROS to interface with a physical CAN bus and the VESC.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros2_can_bridge.CANSimplePubNode:main',
            'listener = ros2_can_bridge.CANSimpleSubNode:main',
            'bridge = ros2_can_bridge.ROS2CANNode:main',
            'vesc_node = ros2_can_bridge.VESCCANNode:main'
        ],
    },
)

from setuptools import setup

package_name = 'ros2_can_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seacat',
    maintainer_email='seacat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
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

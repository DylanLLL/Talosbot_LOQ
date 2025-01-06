from setuptools import find_packages, setup

package_name = 'mqtt_to_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='gdnuser',
    maintainer_email='michaeljonathan664@gmail.com',
    description='Bridge between MQTT and ROS2',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_to_ros_bridge = mqtt_to_ros_bridge.mqtt_to_ros_bridge:main'
        ],
    },
)

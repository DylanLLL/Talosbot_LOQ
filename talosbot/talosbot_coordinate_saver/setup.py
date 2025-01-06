from setuptools import find_packages, setup

package_name = 'talosbot_coordinate_saver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michaeljonathan664@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record = talosbot_coordinate_saver.coordinate_saver:main',
            'record_tf = talosbot_coordinate_saver.coordinate_saver_tf:main',
        ],
    },
)

from setuptools import find_packages
from setuptools import setup

package_name = 'talosbot_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Michael Jonathan',
    author_email='michaeljonathan664@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleop and Locking Pin Control using keyboard for Talosbot'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = talosbot_teleop.script.teleop_keyboard:main'
        ],
    },
)

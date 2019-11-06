import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_follower'

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', 'clf'))),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', 'clf2'))),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', 'turtlebot3_follower_filter.yaml'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_follower.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_follower_filter.launch.py'))),
        ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz', 'turtlebot3_follower.rviz'))),
    ],
    install_requires=['setuptools','launch'],
    zip_safe=True,
    author=['Ryan Shim'],
    author_email=['jhshim@robotis.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'ROS 2 pacakge for turtlebot3_follower.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'turtlebot3_follower = turtlebot3_follower.turtlebot3_follower.main:main',
        ],
    },
)

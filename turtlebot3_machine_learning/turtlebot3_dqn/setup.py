import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_dqn'

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # To be added
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_interactive_marker.launch.py'))),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_obstacle_detection.launch.py'))),
        # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz', 'turtlebot3_interactive_marker.rviz'))),
    ],
    install_requires=['setuptools','launch'],
    zip_safe=True,
    author=['Ryan Shim', 'Gilbert'],
    author_email=['jhshim@robotis.com', 'kkjong@robotis.com'],
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
        'Examples of Different TurtleBot3 Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            # To be added
            'turtlebot3_moving_obstacle = turtlebot3_dqn.turtlebot3_moving_obstacle.turtlebot3_moving_obstacle:main', 
            'turtlebot3_moving_obstacle2 = turtlebot3_dqn.turtlebot3_moving_obstacle.turtlebot3_moving_obstacle2:main', 
        ],
    },
)


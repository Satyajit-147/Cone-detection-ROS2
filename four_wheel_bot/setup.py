from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'four_wheel_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index and metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Install URDF/XACRO files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),

        # Install world files (add this if you're using a world file)
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satyajit',
    maintainer_email='satyajitgr9@gmail.com',
    description='4-Wheel Differential Drive Robot for RViz and Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stopper = obstacle_stop.obstacle_stop_node:main',
            'orange_detector = cone_detector.orange_detector:main',
        ],
    },
)


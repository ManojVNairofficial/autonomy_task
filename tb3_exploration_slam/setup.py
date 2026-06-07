from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_exploration_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manoj',
    maintainer_email='manojvs1969@gmail.com',
    description='Autonomous frontier-based exploration with slam_toolbox and Nav2 for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = tb3_exploration_slam.frontier_explorer:main',
        ],
    },
)

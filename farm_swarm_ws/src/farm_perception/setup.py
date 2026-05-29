from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'farm_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farm Swarm Developer',
    maintainer_email='user@example.com',
    description='Perception modules for farm swarm robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'weed_detector_node     = farm_perception.weed_detector_node:main',
            'weed_removal_node      = farm_perception.weed_removal_node:main',
            'boundary_enforcer_node = farm_perception.boundary_enforcer_node:main',
            'robot1_navigator_node  = farm_perception.robot1_navigator_node:main',
            'convoy_controller_node = farm_perception.convoy_controller_node:main',
        ],
    },
)

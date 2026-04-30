from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'rosbot_traffic_light'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PAR Group',
    maintainer_email='student@rmit.edu.au',
    description='Traffic light behaviour for Husarion ROSbot 3 PRO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_detector = rosbot_traffic_light.traffic_light_detector_node:main',
            'traffic_light_controller = rosbot_traffic_light.traffic_light_controller_node:main',
            'traffic_light_sim = rosbot_traffic_light.traffic_light_sim_node:main',
        ],
    },
)

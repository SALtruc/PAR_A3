from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rosbot_qr_navigation'

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
    description='QR Code Command Navigation – Project A',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detector       = rosbot_qr_navigation.qr_detector_node:main',
            'command_interpreter = rosbot_qr_navigation.command_interpreter_node:main',
            'navigation_fsm    = rosbot_qr_navigation.navigation_fsm_node:main',
            'event_logger      = rosbot_qr_navigation.event_logger_node:main',
            'simulation_driver = rosbot_qr_navigation.simulation_driver_node:main',
        ],
    },
)

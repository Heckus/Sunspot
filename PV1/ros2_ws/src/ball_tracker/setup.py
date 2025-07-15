import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ball_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files from the 'config' directory
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hecke',
    maintainer_email='hecke@todo.todo',
    description='A 3D ball tracker pipeline.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ball_tracker.camera_node:main',
            'detection_node = ball_tracker.detection_node:main',
            'calculation_node = ball_tracker.calculation_node:main',
            'extrinsic_calibrator = ball_tracker.extrinsic_calibration_node:main',
            'battery_monitor = ball_tracker.battery_monitor_node:main',
            'frame_saver = ball_tracker.frame_saver_node:main',
        ],
    },
)
import os
from glob import glob
from setuptools import setup

package_name = 'ball_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- This section installs all your resource files ---
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae'))), # Change to .stl if needed
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Ball tracker package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ball_tracker.camera_node:main',
            'detection_node = ball_tracker.detection_node:main',
            'calculation_node = ball_tracker.calculation_node:main',
            'extrinsic_calibrator = ball_tracker.extrinsic_calibrator:main',
            'frame_saver = ball_tracker.frame_saver:main',
            'battery_monitor = ball_tracker.battery_monitor:main',
        ],
    },
)
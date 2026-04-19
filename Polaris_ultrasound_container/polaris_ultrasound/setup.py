from setuptools import setup
import os
from glob import glob

package_name = 'polaris_ultrasound'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 package for Polaris tracking and ultrasound perception modules',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # core reusable nodes
            'polaris_reader = polaris_ultrasound.polaris_reader_node:main',
            'ultrasound_reader = polaris_ultrasound.us_reader_node:main',
            'us_dataset_player_node = polaris_ultrasound.us_dataset_player_node:main',
            'us_stability_node = polaris_ultrasound.us_stability_node:main',
            'us_target_estimator_node = polaris_ultrasound.us_target_estimator_node:main',

            # workflow-level node
            'calibration_recorder = polaris_ultrasound.calibration_recorder:main',
        ],
    },
)

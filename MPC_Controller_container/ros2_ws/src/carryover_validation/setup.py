from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'carryover_validation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Kewei Zuo',
    maintainer_email='zuo-kewei@g.ecc.u-tokyo.ac.jp',
    description=(
        'Carry-over model validation package with open-loop lateral step '
        'experiments and online RLS estimation of beta_perp.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'validation_runner = carryover_validation.validation_runner_node:main',
            'rls_estimator = carryover_validation.rls_estimator_node:main',
            'target_point_adapter = carryover_validation.target_point_adapter_node:main',
            'experiment_logger = carryover_validation.experiment_logger_node:main',
            'offline_analyzer = carryover_validation.offline_analyzer:main',
            'mock_target_publisher = carryover_validation.mock_target_publisher_node:main',
        ],
    },
)

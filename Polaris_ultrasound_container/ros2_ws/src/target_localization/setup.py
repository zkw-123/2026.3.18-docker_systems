from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'target_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引与包资源
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 与默认参数
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 文档（可选）
        (os.path.join('share', package_name, 'docs'), [f for f in glob('*.md') if 'README' in f]),
    ],
    install_requires=[
        'setuptools',
        # 下面这三项通常用系统包安装；留在此处方便纯 pip 环境
        'numpy',
        'opencv-python',
        'scikit-surgerynditracker'
    ],
    zip_safe=True,
    author='Kewei ZUO',
    author_email='zuo-kewei@g.ecc.u-tokyo.ac.jp',
    maintainer='Kewei ZUO',
    maintainer_email='zuo-kewei@g.ecc.u-tokyo.ac.jp',
    description='Realtime target localization with ultrasound + NDI Polaris.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # —— 文档中出现并且你已提供的节点 —— 
            'ultrasound_publisher    = target_localization.ultrasound_publisher_node:main',
            'polaris_publisher       = target_localization.polaris_publisher_node:main',
            'data_synchronizer       = target_localization.data_synchronizer_node:main',
            'target_detector         = target_localization.target_detector_node:main',
            'target_localizer        = target_localization.target_localizer_node:main',
            'data_recorder           = target_localization.data_recorder_node:main',
            'virtual_point_publisher = target_localization.virtual_point_publisher_node:main',
            # 工具脚本（离线标定）
            'calibration_tool    = target_localization.calibration_tool:main',
            'perception_replay = target_localization.perception_replay_node:main',
           

        ],
    },
)


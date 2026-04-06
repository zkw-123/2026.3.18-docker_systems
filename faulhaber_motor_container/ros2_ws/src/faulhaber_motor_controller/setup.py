from setuptools import setup, find_packages

package_name = 'faulhaber_motor_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    package_dir={'': '.'},  
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_controller_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=[
        'setuptools',
        #'rclpy',
        #'geometry_msgs',
        'pyserial'
    ],
    zip_safe=True,
    maintainer='Kewei Zuo',
    maintainer_email='zkwutknkd@gmail.com',
    description='Basic ROS2 package to control Faulhaber motor via RS232',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_controller_node = faulhaber_motor_controller.motor_controller_node:main',
        ],
    },
)


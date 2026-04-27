from glob import glob

from setuptools import setup

package_name = 'yolo_lidar_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/model', glob('model/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xpy',
    maintainer_email='you@example.com',
    description='YOLO and LiDAR fusion package for ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = yolo_lidar_fusion.fusion_node:main',
        ],
    },
)
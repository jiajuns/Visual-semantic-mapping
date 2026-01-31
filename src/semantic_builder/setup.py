from setuptools import setup
import os
from glob import glob

package_name = 'semantic_builder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jj',
    maintainer_email='jj@todo.todo',
    description='YOLO + RTAB-Map Semantic Builder',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = semantic_builder.yolo_node:main',
            'cloud_saver = semantic_builder.semantic_cloud_saver:main',
            'ply2gltf = semantic_builder.ply2gltf:main',
        ],
    },
)
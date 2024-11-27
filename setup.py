from setuptools import setup
import os
from glob import glob

package_name = 'sendbooster_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sendbooster_server.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/amr.urdf']),
        ('share/' + package_name + '/config', ['config/sendbooster.lua']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='AMR bringup package start server.',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sendbooster_odom = sendbooster_server.sendbooster_odom:main',
            'sendbooster_JointStatePublisher = sendbooster_server.sendbooster_JointStatePublisher:main',
            'tf_broadcaster_node = sendbooster_server.tf_broadcaster_node:main'
        ],
    },
)

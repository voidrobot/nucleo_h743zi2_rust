import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'host_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='void',
    maintainer_email='void@example.com',
    description='Host Bringup and Verification Node for NUCLEO-H743ZI2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ahrs_verifier = host_bringup.ahrs_verifier_node:main',
        ],
    },
)

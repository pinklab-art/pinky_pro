import os
import glob
from setuptools import find_packages, setup

package_name = 'pinky_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*launch.*'))),
        ('share/' + package_name + '/config', glob.glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/worlds', glob.glob(os.path.join('worlds', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pl3',
    maintainer_email='kyung133851@pinklab.art',
    description='Pinky Pro MuJoCo simulator and ROS 2 bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge=pinky_mujoco.bridge:main',
        ],
    },
)

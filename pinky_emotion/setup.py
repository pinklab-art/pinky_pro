from setuptools import find_packages, setup
import os, glob

package_name = 'pinky_emotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/emotion', glob.glob(os.path.join('emotion', '*.gif'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinklab',
    maintainer_email='kyung133851@pinklab.art',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pinky_emotion=pinky_emotion.pinky_emotion:main"
        ],
    },
)
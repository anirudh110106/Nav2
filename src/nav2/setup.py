from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installs all .py files from the launch/ folder
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpd',
    maintainer_email='anirudh110106@gmail.com',
    description='Robot system with odometry and SLAM',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Node executables only — launch files do NOT go here
            'odom = nav2.odom:main',
            'laser_node = nav2.laser:main',
        ],
    },
)

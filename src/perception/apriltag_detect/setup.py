from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltag_detect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='AprilTag detection and global-frame target pose publishing for D435i + OpenVINS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_pose_node = apriltag_detect.target_pose_node:main',
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'edge_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u',
    maintainer_email='erinjhu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Expose the main() function of each node as an executable command
            'state_estimator = edge_robot.state_estimator_node:main',
            'pid = edge_robot.pid_node:main',
            'mock_imu = edge_robot.mock_imu_node:main',
        ],
    },
)

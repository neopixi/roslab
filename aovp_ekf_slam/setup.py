from glob import glob
from setuptools import find_packages, setup


package_name = 'aovp_ekf_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Gazebo front-drive cart with obstacle avoidance and EKF-SLAM.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ekf_slam_node = aovp_ekf_slam.ekf_slam_node:main',
            'motion_controller = aovp_ekf_slam.motion_controller:main',
        ],
    },
)

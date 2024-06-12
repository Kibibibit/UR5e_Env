from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'par_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*')),
        (os.path.join('share', package_name, "srdf"), glob('srdf/*')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        (os.path.join('share', package_name, "rviz"), glob('rviz/*')),
        (os.path.join('share', package_name, "meshes/visual"), glob('meshes/visual/*')),
        (os.path.join('share', package_name, "meshes/collision"), glob('meshes/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Mills',
    maintainer_email='s3843035@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_control_node = par_pkg.gripper_control_node:main',
            'gripper_state_publisher_node = par_pkg.gripper_state_publisher_node:main',
            'move_to_pose_node = par_pkg.move_to_pose_node:main',
            'calibrate_camera_node = par_pkg.calibrate_camera_node:main'
        ],
    },
)

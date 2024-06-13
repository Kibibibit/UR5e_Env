from setuptools import find_packages, setup

from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'onrobot_rg2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
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
            'gripper_control_node = onrobot_rg2_driver.gripper_control_node:main',
            'gripper_state_publisher_node = onrobot_rg2_driver.gripper_state_publisher_node:main',
        ],
    },
)

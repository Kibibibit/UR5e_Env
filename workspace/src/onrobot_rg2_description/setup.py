from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'onrobot_rg2_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*')),
        (os.path.join('share', package_name, "srdf"), glob('srdf/*')),
        (os.path.join('share', package_name, "meshes/visual"), glob('meshes/visual/*')),
        (os.path.join('share', package_name, "meshes/collision"), glob('meshes/collision/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Mills',
    maintainer_email='s3843035@student.rmit.edu.au',
    description='Stores the description files for the onrobot rg2 gripper',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

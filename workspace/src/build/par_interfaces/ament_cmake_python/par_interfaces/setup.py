from setuptools import find_packages
from setuptools import setup

setup(
    name='par_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('par_interfaces', 'par_interfaces.*')),
)

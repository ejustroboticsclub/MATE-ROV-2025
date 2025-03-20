from setuptools import find_packages
from setuptools import setup

setup(
    name='rov_debug_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('rov_debug_interfaces', 'rov_debug_interfaces.*')),
)

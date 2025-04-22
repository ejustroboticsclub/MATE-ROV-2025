from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rov25'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.[pxy][yma]*')))        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaheen',
    maintainer_email='Shaheen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "movement_feedback = rov25.movement_feedback:main",
            "motion_control = rov25.motion_control:main",
            "joystick = rov25.joystick:main",
            "simulate_yaw = rov25.simulate_yaw:main",
            "manual_callibartion = rov25.manual_callibartion:main",
            "blackbox = rov25.blackbox:main",
            "vis = rov25.vis:main"
        ],
    },
)



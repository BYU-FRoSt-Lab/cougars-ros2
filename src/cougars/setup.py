import os
from glob import glob
from setuptools import setup

package_name = 'cougars'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='High-level AUV ROS nodes for use in the BYU FRoSt Lab',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moos_bridge = cougars.moos_bridge:main',
            'dvl_parser = cougars.dvl_parser:main',
            'manual_control = cougars.manual_control:main',
            'leak_sub = cougars.leak_sub:main',
            'battery_sub = cougars.battery_sub:main',
        ],
    },
)
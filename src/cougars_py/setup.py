import os
from glob import glob
from setuptools import setup

package_name = 'cougars_py'

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
            'moos_bridge = cougars_py.moos_bridge:main',
            'dvl_parser = cougars_py.dvl_parser:main',
            'manual_control = cougars_py.manual_control:main',
            'leak_sub = cougars_py.leak_sub:main',
            'battery_sub = cougars_py.battery_sub:main',
            'modem_pinger = cougars_py.modem_pinger:main',
            'seatrac_ahrs_converter = cougars_py.seatrac_ahrs_converter:main',
            'gps_odom = cougars_py.gps_odom:main',
            'factor_graph = cougars_py.factor_graph:main',

        ],
    },
)
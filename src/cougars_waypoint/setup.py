from setuptools import setup
import os
from glob import glob

package_name = 'cougars_waypoint' # CHANGED

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name], # This refers to the Python package directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), # CHANGED (resource file name)
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.mvc')),
    ],
    install_requires=['setuptools', 'pyproj', 'PyYAML'],
    zip_safe=True,
    maintainer='Brighton Anderson',
    maintainer_email='your_email@example.com',
    description='A Mapviz based waypoint tool for Cougars robots.', # Minor tweak
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Python executable name = package_name.module_name:main_function
            'map_point_converter_node = cougars_waypoint.map_point_converter_node:main', # CHANGED
            'waypoint_saver_node = cougars_waypoint.waypoint_saver_node:main', # CHANGED
        ],
    },
)
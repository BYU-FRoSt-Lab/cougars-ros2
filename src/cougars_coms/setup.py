from setuptools import setup

package_name = 'cougars_coms'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/radio_node.py']),  # Add this line
    ],
    install_requires=['setuptools', 'digi-xbee'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='RF Bridge for CoUGARs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radio_node = cougars_coms.radio_node:main'
        ],
    },
)
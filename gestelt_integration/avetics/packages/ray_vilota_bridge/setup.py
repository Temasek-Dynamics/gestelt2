from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vilota_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'capnp'), glob(os.path.join('vilota_bridge/capnp', '*.capnp')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William Leong',
    maintainer_email='tsllwl@nus.edu.sg',
    description='<description>Bridge Vilota eCal and ROS2</description>',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vilota_bridge_node = vilota_bridge.vilota_bridge_node:main'
        ],
    },
)

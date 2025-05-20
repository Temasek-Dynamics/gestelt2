from glob import glob
import os

from setuptools import setup


package_name = 'gestelt_commander'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['scenarios.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stevenmacenski@gmail.com',
    description='An importable library for writing mobile robot applications in python3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = gestelt_commander.mission:main',
            'land = gestelt_commander.land:main',
            'takeoff = gestelt_commander.takeoff:main',
            'send_goals = gestelt_commander.send_goals:main',
            'test_take_off_goal = gestelt_commander.test_take_off_goal:main',
            'test_planning = gestelt_commander.test_planning:main',
            'test_point_goal = gestelt_commander.test_point_goal:main',
        ],
    },
)

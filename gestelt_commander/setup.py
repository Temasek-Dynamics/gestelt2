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
    maintainer_email='johntgz@nus.edu.sg',
    description='Nodes to command take off, landing and execution of goals',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'land = gestelt_commander.land:main',
            'land_sim = gestelt_commander.land_sim:main',
            'test_take_off_goal = gestelt_commander.test_take_off_goal:main',
            'test_take_off_goal_sim = gestelt_commander.test_take_off_goal_sim:main',
            'test_planning = gestelt_commander.test_planning:main',
            'test_point_goal = gestelt_commander.test_point_goal:main',
            'test_point_goal_sim = gestelt_commander.test_point_goal_sim:main',
        ],
    },
)

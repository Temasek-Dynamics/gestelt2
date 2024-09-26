from setuptools import find_packages, setup

package_name = 'gestelt_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='john_tanguanzhong@hotmail.com',
    description='Scripts for executing pre-defined missions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_dbg = gestelt_mission.planner_dbg:main',
        ],
    },
)

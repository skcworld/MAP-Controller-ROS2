from setuptools import setup
import os
from glob import glob

package_name = 'map_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nadine Imholz',
    maintainer_email='nimholz@ethz.ch',
    description='Model- and Acceleration-based Pursuit controller for autonomous racing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_controller = map_controller.MAP_controller:main',
            'pp_controller = map_controller.PP_controller:main',
            'time_error_tracker = map_controller.time_error_tracker:main',
        ],
    },
)

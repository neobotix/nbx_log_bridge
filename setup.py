"""Package the NBX Lemma log bridge for ament."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'nbx_log_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neobotix GmbH',
    maintainer_email='ros@neobotix.de',
    description='Retained warning-and-higher ROS log bridge for NBX Lemma.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nbx_log_bridge = nbx_log_bridge.log_bridge:main',
        ],
    },
)

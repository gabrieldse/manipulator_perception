from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ring_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config',glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='gabriel.oliveira.dse@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_circle = ring_tracker.track_circle:main',
        ],
    },
)

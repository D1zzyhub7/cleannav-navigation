from setuptools import setup
import os
from glob import glob

package_name = 'cleannav_safety_supervisor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d1zzy',
    maintainer_email='d1zzy@example.com',
    description='CleanNav Safety Supervisor — default block_all, zero velocity only',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor_node = '
            'cleannav_safety_supervisor.safety_supervisor_node:main',
        ],
    },
)

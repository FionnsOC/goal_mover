from setuptools import setup
import os
from glob import glob

package_name = 'goal_mover'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Goal mover node with trapezoidal + angular profiles',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'goal_mover_node = goal_mover.goal_mover_node:main',
        ],
    },
)

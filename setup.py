from setuptools import setup
import os
from glob import glob

package_name = 'tree_biomass_rviz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhutvik',
    maintainer_email='rhutvik@example.com',
    description='Tree biomass estimation and RViz2 visualization using ALS point cloud',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'biomass_node = tree_biomass_rviz.biomass_node:main',
        ],
    },
)


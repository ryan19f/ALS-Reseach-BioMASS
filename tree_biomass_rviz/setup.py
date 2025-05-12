from setuptools import setup

package_name = 'tree_biomass_rviz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tree_biomass_rviz']),
        ('share/tree_biomass_rviz', ['package.xml']),
        ('share/tree_biomass_rviz/launch', ['launch/biomass_visualizer.launch.py']),
        ('share/tree_biomass_rviz/config', ['config/biomass_view.rviz']),  # Optional if config exists
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhutvik',
    maintainer_email='rhutvik@example.com',
    description='Tree biomass estimation and visualization in ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'biomass_visualizer_node = tree_biomass_rviz.biomass_visualizer_node:main',
        ],
    },

)

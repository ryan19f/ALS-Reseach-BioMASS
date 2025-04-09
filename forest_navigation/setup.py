from setuptools import setup

package_name = 'forest_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        'gps_normalizer',
        'lidar_filter_node',
        'imu_filter_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/forest_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/lidar_filter.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Forest sim with noise filtering and georeferencing tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_normalizer = gps_normalizer:main',
            'lidar_filter_node = lidar_filter_node:main',
            'imu_filter_node = imu_filter_node:main',
        ],
    },
)

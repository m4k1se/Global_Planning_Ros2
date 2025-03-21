from setuptools import setup

package_name = 'my_mapping_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Mapping and Visualization Package',
    license='Apache License 2.0',
    extras_require={
        'test' : ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'monitor_visualizator = my_mapping_package.monitor_visualizator:main',
            'map_visualizator = my_mapping_package.map_visualizator:main',
            'map_monitor = my_mapping_package.map_monitor:main',
        ],
    },
)


from setuptools import find_packages, setup

package_name = 'my_planning_package'

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
            'lane_waypoint_planner = my_planning_package.lane_waypoint_planner:main',
            'route_visualizator = my_planning_package.route_visualizator:main',
            'pose_goal_publisher = my_planning_package.pose_goal_publisher:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'yasmin_waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Waypoint navigation using YASMIN state machine',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'yasmin_waypoint_navigator = yasmin_waypoint_nav.yasmin_waypoint_navigator:main',
        ],
    },
)

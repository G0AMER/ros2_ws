from setuptools import find_packages, setup

package_name = 'robot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/maps', ['maps/my_map.yaml']),
        ('share/' + package_name + '/maps', ['maps/my_map.pgm']),
        ('share/' + package_name + '/launch', ['launch/nav2_launch.py']),
        ('share/' + package_name + '/launch', ['launch/search_controller.launch.py']),
        ('share/' + package_name + '/rviz_config', ['rviz_config/turtlebot_clone.rviz']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g-amer',
    maintainer_email='gargouri.ameur@enis.tn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_controller = robot_nav.search_controller:main',
        ],
    },
)

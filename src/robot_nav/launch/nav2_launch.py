import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    turtle_clone_pkg = get_package_share_directory('robot_nav')
    nav2_params_file = os.path.join(turtle_clone_pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(turtle_clone_pkg, 'maps', 'my_map.yaml')


    # Nav2 Bringup with Map
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false'
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        #robot_state_publisher,
        nav2_bringup,
        rviz
    ])

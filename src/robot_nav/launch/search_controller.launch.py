import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
    return LaunchDescription([
        # Launch SLAM tool (replace with your SLAM implementation if needed)
        # Node(
        #        package='slam_toolbox',
        #        executable='async_slam_toolbox_node',
        #        name='slam_toolbox',
        #        output='screen',
        #        parameters=[
        #            {'use_sim_time': False},
        #            {'odom_frame': 'odom'},
        #            {'map_frame': 'map'},
        #            {'base_frame': 'base_footprint'},
        #            {'scan_topic': '/scan'},
        #            {'mode': 'mapping'},
        #            {'minimum_travel_distance': 0.05},    # Reduced for more frequent updates
        #            {'minimum_travel_heading': 0.05},     # Reduced for more frequent updates
        #            {'map_update_interval': 0.02},        # Increased to ~50 Hz updates
        #            {'max_laser_range': 25.0},            # Match YDLIDAR TSA range
        #            {'resolution': 0.05},                 # Keep high resolution
        #            {'transform_tolerance': 0.3},         # Increase tolerance for timing
        #            {'correlation_maximum_distance': 2.0}, # Tighten scan matching distance
        #            {'loop_search_maximum_distance': 2.0}, # Tighten loop closure distance
        #            {'do_loop_closing': True},
        #            {'fine_linear_search_range': 0.3},    # Finer search
        #            {'fine_angular_search_range': 0.3}    # Finer angular search
        #        ]
        #    ),

        # Launch Nav2 stack (assumes you already have nav2_bringup working)
        nav2_bringup,

        # Launch your custom SearchController
        Node(
            package='robot_nav',
            executable='search_controller',
            name='search_controller',
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


import pathlib
import yaml


def generate_launch_description():
    yaml_dir = get_package_share_directory("spirit_high_launch")
    config_file = os.path.join(yaml_dir, 'config/safe_scouting.yaml')
    print(config_file)
    # LaunchConfiguration('ros_control_config').perform(context)

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    visualizer_params = config.get('visualizer', {}).get('ros__parameters', {})
    print(visualizer_params)
    leg_measurements_publisher_params = config.get('leg_measurements_publisher', {}).get('ros__parameters', {})
    mapping_params = config.get('mapping_node', {}).get('ros__parameters', {})
    data_collector_params = config.get('data_collector', {}).get('ros__parameters', {})
    reactive_planner_params = config.get('reactive_planner', {}).get('ros__parameters', {})
    safe_optimization_params = config.get('safe_bayesian_optimization_node', {}).get('ros__parameters', {})
    goal_point_publisher_params = config.get('goal_point_publisher', {}).get('ros__parameters', {})
    ground_truth_server_params = config.get('ground_truth_server', {}).get('ros__parameters', {})
    spatial_measurement_pub_params = config.get('spatial_measurement_publisher', {}).get('ros__parameters', {})
    drive_sim_params = config.get('drive_sim', {}).get('ros__parameters', {})


    return LaunchDescription([
         Node(
            package='safe_bayesian_optimization',
             executable='safe_bayesian_optimization_node',
             name='safe_bayesian_optimization_node',
             parameters=[reactive_planner_params, safe_optimization_params],
             output='screen'
             ),
         Node(
             package='safe_bayesian_optimization',
             executable='goal_point_publisher',
             name='goal_point_publisher',
             parameters=[goal_point_publisher_params],
             output='screen'
             ),
         Node(
            package='safe_bayesian_optimization',
            executable='reactive_navigation_node',
            name='reactive_navigation_node',
            parameters=[reactive_planner_params],
            output='screen'
            ),
        Node(
            package='foxglove_visualization',
            executable='leg_measurements_publisher',
            name='leg_measurements_publisher',
            output='screen',
            parameters=[leg_measurements_publisher_params]
        ),
        Node(
            package='safe_scout_simulator',
            executable='drive_sim',
            name='drive_sim',
            output='screen',
            parameters=[drive_sim_params]
        ),
        Node(
            package='mapping_collector',
            executable='data_collector',
            name='data_collector',
            output='screen',
            parameters=[data_collector_params]
        ),

        Node(
            package='mapping_package',
            executable='terrain_mapping_node',
            name='mapping_node',
            output='screen',
            parameters=[mapping_params]
        ),
        Node(
            package='safe_scout_simulator',
            executable='ground_truth_server',
            name='ground_truth_server',
            output='screen',
            parameters=[ground_truth_server_params],
        ),
        Node(
            package='safe_scout_simulator',
            executable='spatial_measurement_publisher.py',
            name='spatial_measurement_publisher',
            output='screen',
            parameters=[spatial_measurement_pub_params],
        ),


    ])

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
    config_file = os.path.join(yaml_dir, 'config/whitesandsafescoutingLABmocaptesting.yaml')
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
    foxglove_bridge_params = config.get('foxglove_bridge', {}).get('ros__parameters', {})
    print("foxglove_bridge_params from config:", foxglove_bridge_params)
    # Convert foxglove_bridge_params to launch arguments format
    launch_args = {}
    for key, value in foxglove_bridge_params.items():
        if isinstance(value, list):
            # Convert list to string format for launch arguments
            # Example: ['topic1', 'topic2'] -> [topic1,topic2]
            launch_args[key] = str(value).replace("'", "").replace(" ", "")
        else:
            # Convert other types to strings
            launch_args[key] = str(value)
    
    print("Converted launch arguments:", launch_args)
    
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        ]),
        launch_arguments=launch_args.items()
    )


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
        # Node(
        #     package='foxglove_visualization',  # Replace with the package where FakeDataPublisher is defined
        #     executable='leg_measurements_publisher',  # Replace with the executable name of FakeDataPublisher
        #     name='leg_measurements_publisher',
        #     output='screen',
        #     parameters=[leg_measurements_publisher_params]
        # ),
        # Node(
        #     package='foxglove_visualization',  # Replace with the package where FakeDataPublisher is defined
        #     executable='fake_data_publisher',  # Replace with the executable name of FakeDataPublisher
        #     name='fake_data_publisher',
        #     output='screen'
        # ),
        # Node(
        #     package='foxglove_visualization',  # Replace with the package where FakeDataPublisher is defined
        #     executable='drive_sim',  # Replace with the executable name of FakeDataPublisher
        #     name='drive_sim',
        #     output='screen'
        # ),
        # Node(
        #     package='mapping_collector',  # Replace with the package where FakeDataPublisher is defined
        #     executable='data_collector',  # Replace with the executable name of FakeDataPublisher
        #     name='data_collector',
        #     output='screen',
        #     parameters=[data_collector_params]
        # ),

        # Node(
        #     package='mapping_package',  # Replace with the package where FakeDataPublisher is defined
        #     executable='terrain_mapping_node',  # Replace with the executable name of FakeDataPublisher
        #     name='mapping_node',
        #     output='screen',
        #     parameters=[mapping_params]
        # ),
        # Node(
        #     package='turtlesim',
        #     executable='turtlesim_node',
        #     name='turtlesim_node',
        #     output='screen',
        #     parameters=[{'background_r': 255, 'background_g': 255, 'background_b': 255}]
        #     ),
        Node(
            package='safe_bayesian_optimization',
            executable='turtlesim_spatial_publisher.py',
            name='turtlesim_spatial_publisher',
            output='screen'
            ),

        # foxglove_bridge_launch,


    ]) 

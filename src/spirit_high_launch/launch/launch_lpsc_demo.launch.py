from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import pathlib
import yaml


def generate_launch_description():
    yaml_dir = "/home/qianlab/SpiritHighLevel/src/spirit_high_launch/config"
    config_file = os.path.join(yaml_dir, 'lpsc.yaml')
    print(config_file)
    # LaunchConfiguration('ros_control_config').perform(context)
    
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    visualizer_params = config.get('visualizer', {}).get('ros__parameters', {})
    print(visualizer_params)
    mapping_params = config.get('mapping_node', {}).get('ros__parameters', {})
    data_collector_params = config.get('data_collector', {}).get('ros__parameters', {})
    
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        ])
    )

    return LaunchDescription([ 
        # Node 2: Foxglove
        Node(
            package='foxglove_visualization',  # Replace with the package where Foxglove is defined
            executable='visualizer',  # Replace with the executable name of Foxglove
            name='visualizer',
            output='screen',
            parameters=[visualizer_params]
            
        ),

        Node(
            package='foxglove_visualization',  # Replace with the package where Foxglove is defined
            executable='mocap_collection',  # Replace with the executable name of Foxglove
            name='mocap_collection',
            output='screen',
            parameters=[visualizer_params]
            
        ),


        Node(
            package='foxglove_visualization',  # Replace with the package where FakeDataPublisher is defined
            executable='leg_measurements_publisher',  # Replace with the executable name of FakeDataPublisher
            name='leg_measurements_publisher',
            output='screen'
        ),


        Node(
            package='top_view_visualization',  # Replace with the package where FakeDataPublisher is defined
            executable='camera',  # Replace with the executable name of FakeDataPublisher
            name='top_view_camera',
            output='screen'
        ),
        
        Node(
            package='mapping_collector',  # Replace with the package where FakeDataPublisher is defined
            executable='data_collector',  # Replace with the executable name of FakeDataPublisher
            name='data_collector',
            output='screen',
            parameters=[data_collector_params]
        ),

        Node(
            package='mapping_package',  # Replace with the package where FakeDataPublisher is defined
            executable='mapping_node',  # Replace with the executable name of FakeDataPublisher
            name='mapping_node',
            output='screen',
            parameters=[mapping_params]
        ),

        foxglove_bridge_launch,


    ])

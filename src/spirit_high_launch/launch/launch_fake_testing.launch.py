from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():

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
        # Node 1: FakeDataPublisher
        Node(
            package='foxglove_visualization',  # Replace with the package where FakeDataPublisher is defined
            executable='fake_data_publisher',  # Replace with the executable name of FakeDataPublisher
            name='fake_data_publisher',
            output='screen'
        ),
        
        # Node 2: Foxglove
        Node(
            package='foxglove_visualization',  # Replace with the package where Foxglove is defined
            executable='visualizer',  # Replace with the executable name of Foxglove
            name='visualizer',
            output='screen'
        ),

        foxglove_bridge_launch,

        Node(
            package='top_view_visualization', 
            executable='camera', 
            name='camera',
            output='screen'
        ),
    ])

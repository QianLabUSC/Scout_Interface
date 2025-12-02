from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    yaml_dir = get_package_share_directory("spirit_high_launch")
    config_file = os.path.join(yaml_dir, 'config/safe_scouting.yaml')
    print(config_file)

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

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
        foxglove_bridge_launch,
    ])

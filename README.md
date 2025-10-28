# Scout Interface - Safe Exploration Deformable System

A comprehensive ROS 2 system for safe robotic exploration and terrain mapping using Bayesian optimization, risk mapping, and reactive navigation.

## System Overview

The Scout Interface integrates four main components:
1. **Scout Interface** - Main visualization and launch system
2. **Risk Mapping** - Terrain mapping and data collection system
3. **Safe Legged Scouting** - Safe Bayesian optimization and reactive navigation
4. **TRUSSES Custom Interfaces** - Custom ROS 2 message and service definitions

## Repository Structure

```
your_ros_workspace/src/
├── Scout_Interface/           # Main scout interface (this repository)
├── risk_mapping/             # Risk mapping and terrain analysis
├── safe_legged_scouting/     # Safe Bayesian optimization and navigation
├── trusses_custom_interfaces/ # Custom ROS 2 message and service interfaces
└── dense-ground-truth-generator/  # Ground truth data generation
```

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04)
- **Python 3.10+**
- **C++14 compiler**
- **Git**

### System Dependencies

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool -y

# Install additional dependencies
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-opencv \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libcgal-dev \
    libgdal-dev \
    cmake \
    build-essential
sudo apt update && sudo apt install -y libcurl4-openssl-dev libgdal-dev libnetcdf-dev
# Initialize rosdep
sudo rosdep init
rosdep update
```

## Installation

### 1. Clone All Required Repositories

```bash
# Create main workspace directory (replace 'your_ros_workspace' with your desired workspace name)
mkdir -p ~/your_ros_workspace/src
cd ~/your_ros_workspace/src/

# Clone the main Scout Interface repository
git clone -b main git@github.com:QianLabUSC/Scout_Interface.git

# Clone the Risk Mapping repository
git clone -b mapping_world_coordinates git@github.com:TRUSSES/risk_mapping.git

# Clone the Safe Legged Scouting repository
git clone git@github.com:QianLabUSC/safe_legged_scouting.git

# Clone the TRUSSES Custom Interfaces repository
git clone git@github.com:TRUSSES/trusses_custom_interfaces.git

# Clone the Dense Ground Truth Generator (optional)
git clone <dense-ground-truth-repo-url> dense-ground-truth-generator
```

### 2. Build the Workspace

```bash
# Navigate to the main workspace
cd ~/your_ros_workspace

# Build all packages
colcon build --cmake-args -DBUILD_EXAMPLES=OFF

# Source the workspace
source install/setup.bash
```

### 3. Install Python Dependencies

```bash
# Install Python packages for risk mapping
cd risk_mapping
pip3 install -r requirements.txt

# Install additional Python dependencies
pip3 install \
    scikit-learn \
    gpytorch \
    torch \
    matplotlib \
    opencv-python \
    pandas \
    scipy
```

## Quick Start

### 1. Basic Scout Interface Launch (for lab demo, robot is controlled by joystick)

```bash
# Source the workspace
cd ~/your_ros_workspace
source install/setup.bash

# Launch the basic scout interface with Foxglove visualization
ros2 launch spirit_high_launch launch_ws_scouting_foxglove.launch.py

# open another terminal. 
ros2 launch spirit_high_launch launch_ws_scouting_maping.launch.py

# open another terminal. 
ros2 launch spirit_high_launch launch_ws_scouting_robotsciencenode.launch.py
```


### 2. Safe Bayesian Optimization System

```bash
# Launch the complete safe scouting system
ros2 launch spirit_high_launch safe_bayesian_optimization_full.launch.py
```

### 3. Risk Mapping System

```bash
# Launch the terrain mapping system
ros2 launch spirit_high_launch safe_bayesian_optimization_mapping_launch.py
```

## Configuration

### Scout Interface Configuration

Edit configuration files in `Scout_Interface/src/spirit_high_launch/config/`:

- `whitesandsafescouting.yaml` - Main safe scouting configuration
- `whitesandrover.yaml` - Rover-specific parameters
- `rss.yaml` - RSS (Risk-Sensitive Scouting) configuration

### Risk Mapping Configuration

Configure terrain mapping parameters in `risk_mapping/src/mapping_package/`:
- GPR parameters (noise level, length scale, sigma)
- Map resolution and bounds
- Data collection settings

### Safe Legged Scouting Configuration

Configure optimization parameters in `safe_legged_scouting/config/`:
- `safe_bayesian_optimization.yaml` - Bayesian optimization parameters
- `reactive_planner.yaml` - Reactive navigation parameters

## Visualization

### Foxglove Studio Integration

1. Launch the scout interface with Foxglove support
2. Open Foxglove Studio
3. Connect to `ws://localhost:8765`
4. Load the appropriate layout from `Scout_Interface/foxglove_template/`

Available layouts:
- `lassie-spirit-spirit_scouting.json` - Main scouting visualization
- `lassie-spirit-RSS-demo.json` - RSS demonstration
- `trusses_rover_operation_whitesand.json` - Whitesand rover operations

## System Components

### Scout Interface (`Scout_Interface/`)

**Main Components:**
- `foxglove_visualization/` - Real-time visualization nodes
- `scientific_payloads/` - Camera and sensor interfaces
- `spirit_high_launch/` - Launch files and configurations
- `top_view_visualization/` - Top-down camera visualization

**Key Features:**
- Real-time data visualization
- Multi-camera support
- Foxglove Studio integration
- Configurable launch parameters

### Risk Mapping (`risk_mapping/`)

**Main Components:**
- `mapping_collector/` - Data collection and storage
- `mapping_package/` - Terrain and velocity mapping
- `robot_package/` - Robot interface nodes

**Key Features:**
- Gaussian Process Regression (GPR) terrain mapping
- Spatial measurement collection
- Velocity map generation
- CSV data export/import

### Safe Legged Scouting (`safe_legged_scouting/`)

**Main Components:**
- `safe_bayesian_optimization_node` - Main optimization algorithm
- `reactive_navigation_node` - Reactive path planning
- `goal_point_publisher` - Goal point management

**Key Features:**
- Safe Bayesian optimization
- Uncertainty-aware navigation
- Diffeomorphism-based obstacle avoidance
- Real-time subgoal computation

### TRUSSES Custom Interfaces (`trusses_custom_interfaces/`)

**Main Components:**
- Custom ROS 2 message types for spatial measurements and terrain mapping
- Service definitions for data collection and terrain analysis
- Spirit robot-specific message types for commanding and state reporting

**Key Features:**
- `SpatialMeasurement.msg` - Spatial measurement data structure
- `ExtrapolatedMap.msg` - Terrain map data with uncertainty
- `GetTerrainMap.srv` - Service for terrain map requests
- `GetTerrainMapWithUncertainty.srv` - Service for uncertainty-aware terrain maps
- Spirit robot message types for low-level control and state reporting
- Standardized interfaces across TRUSSES project components

**Important Notes:**
- This package is critical for all TRUSSES project components
- Changes require pull request review process
- Must build cleanly across all systems (robots, base stations, laptops)
- Contact maintainers for issues: Shivangi Misra, Diego Caporale, Shipeng Liu, Wilson Hu

## Launch Files

### Scout Interface Launches

```bash
# Basic scouting with Foxglove
ros2 launch spirit_high_launch launch_ws_scouting_foxglove.launch.py

# Full scouting system
ros2 launch spirit_high_launch launch_ws_scouting_full.launch.py

# Mapping only
ros2 launch spirit_high_launch launch_ws_scouting_maping.launch.py
```

### Safe Scouting Launches

```bash
# Complete safe Bayesian optimization system
ros2 launch spirit_high_launch safe_bayesian_optimization_full.launch.py

# Mapping and data collection
ros2 launch spirit_high_launch safe_bayesian_optimization_mapping_launch.py

# Safe planner only
ros2 launch spirit_high_launch safe_bayesian_optimization_safe_planner.launch.py

# Simulation environment
ros2 launch spirit_high_launch safe_bayesian_optimization_simulation.launch.py

# Visualization only
ros2 launch spirit_high_launch safe_bayesian_optimization_visualization.launch.py
```

## Data Flow

1. **Data Collection**: Robot sensors collect spatial measurements
2. **Risk Mapping**: GPR generates terrain maps with uncertainty estimates
3. **Safe Optimization**: Bayesian optimization selects safe subgoals
4. **Reactive Navigation**: Diffeomorphism-based path planning avoids obstacles
5. **Visualization**: Real-time display in Foxglove Studio

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure all dependencies are installed and ROS 2 is properly sourced
2. **Missing Topics**: Check that all required nodes are running
3. **Visualization Issues**: Verify Foxglove connection and layout files
4. **Permission Errors**: Ensure proper SSH keys for GitHub repositories

### Debug Commands

```bash
# Check node status
ros2 node list

# Check topic information
ros2 topic list
ros2 topic info /current_subgoal

# Check service availability
ros2 service list
ros2 service call /get_spatial_data trusses_custom_interfaces/srv/SpatialData

# Monitor system performance
ros2 topic hz /spatial_measurements
```

## Development

### Adding New Features

1. Create new ROS 2 packages in appropriate directories
2. Update launch files to include new nodes
3. Add configuration parameters to YAML files
4. Update Foxglove layouts for new visualizations

### Testing

```bash
# Run tests for individual packages
colcon test --packages-select <package_name>

# Run all tests
colcon test

# View test results
colcon test-result --all --verbose
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes and test thoroughly
4. Submit a pull request with detailed description

## License

This project is licensed under the Apache 2.0 License - see the individual package LICENSE files for details.

## Support

For issues and questions:
- Check the troubleshooting section above
- Review individual package README files
- Open an issue in the appropriate repository

## Acknowledgments

- TRUSSES Lab for the risk mapping system
- Safe Bayesian optimization implementation
- ROS 2 community for excellent tooling and documentation

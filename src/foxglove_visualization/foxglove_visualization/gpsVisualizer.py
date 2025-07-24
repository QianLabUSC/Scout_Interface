import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from trusses_custom_interfaces.msg import SpiritGPS


class GPSPathVisualizer(Node):
    def __init__(self):
        super().__init__('gps_path_visualizer')
        
        # Publishers for path visualization
        self.path_publisher = self.create_publisher(Path, '/spirit/gps/robot_path', 10)
        
        
        # Publisher for mocap pose
        self.mocap_publisher = self.create_publisher(Pose, '/spirit/mocap', 10)
        
        # Subscriber to GPS data
        self.gps_subscriber = self.create_subscription(
            SpiritGPS,
            '/spirit/gps',
            self.gps_callback,
            10
        )
        
        # Initialize path for robot trajectory
        self.robot_path = Path()
        self.robot_path.header.frame_id = "map"
        
        # Reference point for local coordinate conversion (average of first 5 readings)
        self.reference_lat = None
        self.reference_lon = None
        self.reference_alt = None
        
        # Store first 5 GPS readings for averaging
        self.initial_gps_readings = []
        self.reference_samples_needed = 5
        self.reference_set = False
        self.count = 0
        self.get_logger().info('GPS Path Visualizer initialized - Green start, Red current, Blue path')
    
    def set_reference_point(self, lat, lon, alt):
        """Collect and average the first 5 GPS readings for reference point"""
        if self.reference_set:
            return True
            
        # Add current reading to the collection
        self.initial_gps_readings.append({
            'lat': lat,
            'lon': lon, 
            'alt': alt
        })
        
        self.get_logger().info(f'Collecting reference point data: {len(self.initial_gps_readings)}/{self.reference_samples_needed}')
        
        # Check if we have enough samples
        if len(self.initial_gps_readings) >= self.reference_samples_needed:
            # Calculate averages
            avg_lat = sum(reading['lat'] for reading in self.initial_gps_readings) / len(self.initial_gps_readings)
            avg_lon = sum(reading['lon'] for reading in self.initial_gps_readings) / len(self.initial_gps_readings)
            avg_alt = sum(reading['alt'] for reading in self.initial_gps_readings) / len(self.initial_gps_readings)
            
            # Set reference point
            self.reference_lat = avg_lat
            self.reference_lon = avg_lon
            self.reference_alt = avg_alt
            self.reference_set = True
            
            self.get_logger().info(f'Reference point set (averaged from {len(self.initial_gps_readings)} readings):')
            self.get_logger().info(f'  Lat: {self.reference_lat:.8f}')
            self.get_logger().info(f'  Lon: {self.reference_lon:.8f}')
            self.get_logger().info(f'  Alt: {self.reference_alt:.3f}m')
            
            return True
        
        return False
    
    def gps_to_local_coordinates(self, lat, lon, alt):
        """Convert GPS coordinates to local Cartesian coordinates"""
        if not self.reference_set:
            # Still collecting reference point data
            if not self.set_reference_point(lat, lon, alt):
                return 0.0, 0.0, 0.0
        
        # Convert lat/lon differences to meters
        lat_diff = lat - self.reference_lat
        lon_diff = lon - self.reference_lon
        alt_diff = alt - self.reference_alt
        
        # Convert to meters (approximate)
        x = lat_diff * 111320.0  # meters per degree latitude
        y = lon_diff * 111320.0 * math.cos(math.radians(self.reference_lat))  # adjust for longitude
        z = alt_diff
        
        return x, y, z
    
    
    
    def publish_mocap_pose(self, x, y, z):
        """Publish robot pose to mocap topic"""
        mocap_pose = Pose()
        
        # Set position
        mocap_pose.position.x = x
        mocap_pose.position.y = y
        mocap_pose.position.z = z
        
        # Set orientation to zero (no rotation)
        mocap_pose.orientation.x = 0.0
        mocap_pose.orientation.y = 0.0
        mocap_pose.orientation.z = 0.0
        mocap_pose.orientation.w = 1.0
        
        self.mocap_publisher.publish(mocap_pose)
        

        
    
    def gps_callback(self, gps_msg):
        """Process GPS message and create path visualization"""
        try:
            # Convert GPS to local coordinates
            x, y, z = self.gps_to_local_coordinates(
                gps_msg.latitude, 
                gps_msg.longitude, 
                gps_msg.altitude
            )
            # x += self.count
            # y += self.count
            # z = z + self.count
            # self.count += 0.1
            # Skip visualization if still collecting reference point data
            if not self.reference_set:
                return
            
            # Publish current position to mocap topic
            self.publish_mocap_pose(x, y, z)
            
            # Create pose for path
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.w = 1.0  # No rotation
            
            # Add to path
            self.robot_path.poses.append(pose_msg)
            self.robot_path.header.stamp = self.get_clock().now().to_msg()
            
            # Limit path length to avoid memory issues
            if len(self.robot_path.poses) > 1000:
                self.robot_path.poses = self.robot_path.poses[-500:]
            
            # Publish path
            self.path_publisher.publish(self.robot_path)
            
            
            
            # Log status
            self.get_logger().info(
                f'Path point {len(self.robot_path.poses)}: ({x:.2f}, {y:.2f}, {z:.2f}m) | '
                f'GPS: ({gps_msg.latitude:.6f}, {gps_msg.longitude:.6f}) | '
                f'Accuracy: {gps_msg.hacc/1000.0:.2f}m | '
                f'Mocap published'
            )
                
        except Exception as e:
            self.get_logger().error(f'Error processing GPS data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    gps_visualizer = GPSPathVisualizer()
    
    try:
        rclpy.spin(gps_visualizer)
    except KeyboardInterrupt:
        gps_visualizer.get_logger().info('GPS Path Visualizer shutting down')
    finally:
        gps_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
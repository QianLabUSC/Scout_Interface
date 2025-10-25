import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from scipy.spatial.transform import Rotation
import math


class FakePathPublisher(Node):
    def __init__(self):
        super().__init__('fake_path_publisher')
        
        # Declare parameters with default values
        self.declare_parameter('start_position', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_position', [5.0, 5.0, 0.0])
        self.declare_parameter('num_points', 50)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('frame_id', 'map')
        
        # Get parameter values
        start_position = self.get_parameter('start_position').get_parameter_value().double_array_value
        goal_position = self.get_parameter('goal_position').get_parameter_value().double_array_value
        
        # Extract individual coordinates
        self.start_x, self.start_y, self.start_z = start_position[0], start_position[1], start_position[2]
        self.goal_x, self.goal_y, self.goal_z = goal_position[0], goal_position[1], goal_position[2]
        
        self.num_points = self.get_parameter('num_points').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Create publisher
        self.path_publisher = self.create_publisher(Path, 'rover/planned_path', 10)
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_path)
        
        # Generate the smooth path
        self.path = self.generate_smooth_path()
        
        self.get_logger().info(f'Fake Path Publisher started')
        self.get_logger().info(f'Start: ({self.start_x}, {self.start_y}, {self.start_z})')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, {self.goal_z})')
        self.get_logger().info(f'Number of points: {self.num_points}')
        self.get_logger().info(f'Publishing rate: {self.publish_rate} Hz')
    
    def generate_smooth_path(self):
        """Generate a smooth path using cubic spline interpolation"""
        # Create waypoints for a smooth curve
        # Add intermediate control points for a more natural path
        start = np.array([self.start_x, self.start_y, self.start_z])
        goal = np.array([self.goal_x, self.goal_y, self.goal_z])
        
        # Calculate distance and direction
        direction = goal - start
        distance = np.linalg.norm(direction[:2])  # 2D distance
        
        # Create control points for a smooth curve
        # Add some curvature by offsetting intermediate points
        mid_point = (start + goal) / 2
        
        # Add perpendicular offset for curve (optional)
        if distance > 1e-6:  # Avoid division by zero
            perp_vector = np.array([-direction[1], direction[0], 0])
            perp_vector = perp_vector / np.linalg.norm(perp_vector[:2]) if np.linalg.norm(perp_vector[:2]) > 1e-6 else np.array([0, 0, 0])
            curve_offset = perp_vector * (distance * 0.1)  # 10% of distance as curve
            mid_point += curve_offset
        
        # Control points for cubic interpolation
        control_points = np.array([start, mid_point, goal])
        
        # Generate smooth path using parametric interpolation
        path_points = []
        for i in range(self.num_points):
            t = i / (self.num_points - 1)  # Parameter from 0 to 1
            
            # Quadratic Bezier curve interpolation
            point = ((1-t)**2 * control_points[0] + 
                    2*(1-t)*t * control_points[1] + 
                    t**2 * control_points[2])
            
            path_points.append(point)
        
        return np.array(path_points)
    
    def calculate_orientation(self, current_point, next_point):
        """Calculate orientation quaternion based on path direction"""
        if next_point is not None:
            # Calculate yaw from direction to next point
            dx = next_point[0] - current_point[0]
            dy = next_point[1] - current_point[1]
            yaw = math.atan2(dy, dx)
        else:
            # For the last point, use the same orientation as the previous segment
            yaw = 0.0
        
        # Create quaternion from yaw (rotation around z-axis)
        r = Rotation.from_euler('z', yaw)
        quat = r.as_quat()  # Returns [x, y, z, w]
        
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def publish_path(self):
        """Publish the generated path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        
        # Create pose stamped messages for each point
        for i, point in enumerate(self.path):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = self.frame_id
            
            # Set position
            pose_stamped.pose.position = Point(x=point[0], y=point[1], z=point[2])
            
            # Calculate orientation based on path direction
            next_point = self.path[i + 1] if i < len(self.path) - 1 else None
            pose_stamped.pose.orientation = self.calculate_orientation(point, next_point)
            
            path_msg.poses.append(pose_stamped)
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        self.get_logger().debug(f'Published path with {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    fake_path_publisher = FakePathPublisher()
    
    try:
        rclpy.spin(fake_path_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fake_path_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
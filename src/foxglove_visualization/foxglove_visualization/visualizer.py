import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
from scipy.spatial.transform import Rotation
import rclpy
from trusses_custom_interfaces.msg import ExtrapolatedMap, MeasurementArray
from std_msgs.msg import Header
from foxglove_msgs.msg import Grid, PackedElementField, Vector2
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from foxglove_msgs.msg import LocationFix
class Foxglove(Node):
    def __init__(self):
        super().__init__('foxglove_visualization_inter_')
        
        # QoS profile with KEEP_LAST history
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        
        self.spatial_map_subscriber = self.create_subscription(ExtrapolatedMap, 'extrapolated_map', self.spatial_map_callback, self.qos_profile)
        self.spatial_points_subscriber = self.create_subscription(MeasurementArray, 'collected_measurements', self.spatial_points_callback, self.qos_profile)
        self.robot_center_subscriber = self.create_subscription(Pose, 'spirit/mocap', self.mocap_callback, self.qos_profile)
        self.path_subscriber = self.create_subscription(Path, 'planner_path/rover', self.path_callback, self.qos_profile)
        self.gps_path_subscriber = self.create_subscription(Path, '/spirit/gps/robot_path', self.gps_path_callback, self.qos_profile)
        # Publishers for buffer size
        self.terrain_map_publisher = self.create_publisher(Grid, 'terrain_map', 10)
        self.measurements_publisher = self.create_publisher(MarkerArray, 'measurements_markers', 10)
        self.colorbar_publisher = self.create_publisher(Image, 'terrain_color_bar', 10)
        self.robot_regid_body_publisher = self.create_publisher(Marker, 'robot_body', 10)
        self.foxglove_path_publisher = self.create_publisher(MarkerArray, 'foxglove_path_markers', 10)
        self.path_foxglove_publisher = self.create_publisher(Path, '/foxglove/spirit/path', 10)
        self.bridge = CvBridge()
        

        # Declare parameters with default values
        self.declare_parameter('x_range', (-1.5, 1.5))
        self.declare_parameter('y_range', (-3.0, 3.0))
        self.declare_parameter('resolution', (100, 50))
        self.declare_parameter('stiffness_range', (0, 2000))
        self.declare_parameter('uncertainty_threshold', 0.7)
        self.declare_parameter('uncertainty_alpha', 100)

        # Retrieve parameters (defaults will be used if not overridden)
        x_range = self.get_parameter('x_range').value
        y_range = self.get_parameter('y_range').value
        resolution = self.get_parameter('resolution').value
        stiffness_range = self.get_parameter('stiffness_range').value
        uncertainty_threshold = self.get_parameter('uncertainty_threshold').value
        uncertainty_alpha = self.get_parameter('uncertainty_alpha').value

        # Log parameters to confirm values
        self.get_logger().info(f"x_range: {x_range}")
        self.get_logger().info(f"y_range: {y_range}")
        self.get_logger().info(f"resolution: {resolution}")
        self.get_logger().info(f"stiffness_range: {stiffness_range}")
        self.get_logger().info(f"uncertainty_threshold: {uncertainty_threshold}")
        self.get_logger().info(f"uncertainty_alpha: {uncertainty_alpha}")

        self.x_range = (x_range[0], x_range[1])
        self.y_range=  (y_range[0], y_range[1])
        self.resolution = (resolution[0], resolution[1])
        self.stiffness_range = stiffness_range
        self.uncertainty_threshold = uncertainty_threshold
        self.uncertainty_alpha = uncertainty_alpha
        self.last_published_markers = {}  # Key: (ns, id), Value: Marker


    def gps_path_callback(self, msg: Path):
        scaled = Path()
        scaled.header.frame_id = msg.header.frame_id
        # stamp with now or preserve original stamp:
        scaled.header.stamp    = self.get_clock().now().to_msg()

        for ps in msg.poses:
            

            # scale X
            ps.pose.position.x = (
                (ps.pose.position.x - self.x_range[0])
                / (self.x_range[1] - self.x_range[0])
                * self.resolution[1]
            )
            # scale Y
            ps.pose.position.y = (
                (ps.pose.position.y - self.y_range[0])
                / (self.y_range[1] - self.y_range[0])
                * self.resolution[0]
            )
            # scale Z
            ps.pose.position.z = ps.pose.position.z * 10

            # preserve orientation
            ps.pose.orientation = ps.pose.orientation

            scaled.poses.append(ps)

        # finally publish the scaled path
        self.path_foxglove_publisher.publish(scaled)
        


    def path_callback(self, msg: Path):
        """Converts Path data to MarkerArray for visualization with scaled coordinates."""
        path_markers = MarkerArray()
        
        marker_id = 0
        for i, pose_stamped in enumerate(msg.poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Scale coordinates to match map visualization
            # Map: x_range maps to resolution[1] (320), y_range maps to resolution[0] (480)
            scaled_x = (pose_stamped.pose.position.x - self.x_range[0]) / (self.x_range[1] - self.x_range[0]) * self.resolution[1]
            scaled_y = (pose_stamped.pose.position.y - self.y_range[0]) / (self.y_range[1] - self.y_range[0]) * self.resolution[0]
            
            marker.pose.position.x = scaled_x
            marker.pose.position.y = scaled_y
            marker.pose.position.z = pose_stamped.pose.position.z * 10.0  # Scale Z for visibility
            
            # Set marker size and color based on position in path
            if i == 0:  # Start point
                marker.scale.x = 20.0  # Larger marker for start
                marker.scale.y = 20.0
                marker.scale.z = 4.0
                marker.color.r = 0.0
                marker.color.g = 1.0  # Green for start
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.ns = "path_start"
            elif i == len(msg.poses) - 1:  # Goal point
                marker.scale.x = 20.0  # Larger marker for goal
                marker.scale.y = 20.0
                marker.scale.z = 4.0
                marker.color.r = 1.0  # Red for goal
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.ns = "path_goal"
            else:  # Path points
                marker.scale.x = 10.0  # Smaller markers for path
                marker.scale.y = 10.0
                marker.scale.z = 10.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0  # Blue for path
                marker.color.a = 0.8
                marker.ns = "path_waypoint"
            
            path_markers.markers.append(marker)
            marker_id += 1
        
        # Add path lines connecting waypoints
        if len(msg.poses) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "path_line"
            line_marker.id = marker_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Line properties
            line_marker.scale.x = 4.0  # Line width
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0  # Yellow line
            line_marker.color.a = 0.8
            
            # Add all path points to the line
            for pose_stamped in msg.poses:
                point = Point()
                point.x = (pose_stamped.pose.position.x - self.x_range[0]) / (self.x_range[1] - self.x_range[0]) * self.resolution[1]
                point.y = (pose_stamped.pose.position.y - self.y_range[0]) / (self.y_range[1] - self.y_range[0]) * self.resolution[0]
                point.z = pose_stamped.pose.position.z * 10.0
                line_marker.points.append(point)
            
            path_markers.markers.append(line_marker)
        
        # Publish path markers
        self.foxglove_path_publisher.publish(path_markers)
    def mocap_callback(self, msg: Pose):
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame of reference
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rectangle"
        marker.id = 0
        marker.type = Marker.SPHERE
        # marker.action = Marker.ADD
        marker.action = Marker.ADD


        mocap_q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        p_WMo_W = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        
        # Init Rotations
        # quaternion to rotation matrix, this is rotation matrix from MoCap to World
        R_WM = Rotation.from_quat(mocap_q).as_matrix()
        R_MB = np.array([[0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0]])
        R_WB = R_WM @ R_MB

        p_BM_B = np.array([0.037,0,0.1075]) #body to tracker in body
        p_WB_W = p_WMo_W + R_WB @ ( -p_BM_B )
        
        self.R_WB = R_WB
        # print(R_WB)
        # self.CoM_pos = np.array([msg.position.x, msg.position.y, msg.position.z]) + p_offset
        self.CoM_pos = p_WB_W
        # Position
        marker.pose.position.x = (mocap_q[0] - self.x_range[0])/(self.x_range[1] - self.x_range[0]) * self.resolution[1]
        marker.pose.position.y = (1.54 - mocap_q[2] - self.y_range[0])/(self.y_range[1] - self.y_range[0]) * self.resolution[0]
            
        marker.pose.position.z = (mocap_q[1] + 0.1) * 5.0
        
        # Orientation
        # marker.pose.orientation = msg.orientation
        
        # Dimensions of the rectangle
        # marker.scale.x = 50/4 * 0.4  # Length
        # marker.scale.y = 100/6 * 0.6  # Width
        # marker.scale.z = 0.2 * 7  # Thickness

        marker.scale.x = 480/4 * 0.4
        marker.scale.y = 480/4 * 0.4
        marker.scale.z = 80/4 * 0.4


        # Color (gray with transparency)
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.8  # Transparency (0.0 = fully transparent, 1.0 = fully opaque)

        self.robot_regid_body_publisher.publish(marker)
        

    def spatial_map_callback(self, msg: ExtrapolatedMap):
        """Converts ExtrapolatedMap data to a Foxglove Grid for visualization."""
        # Create a Grid message
        grid_msg = Grid()
        height, width = msg.height, msg.width  # Number of rows and columns
        uncertainty = msg.uncertainty
        # Fill in the header
        grid_msg.timestamp = self.get_clock().now().to_msg()
        grid_msg.frame_id = "map"

        # Define the pose of the grid's origin
        grid_msg.pose = Pose()
        grid_msg.pose.position.x = 0.0
        grid_msg.pose.position.y = 0.0
        grid_msg.pose.position.z = 0.0
        grid_msg.pose.orientation.w = 1.0

        # Set grid dimensions and cell size
        grid_msg.column_count = width 
        grid_msg.cell_size = Vector2(x=1.0, y=1.0)  # Adjust cell size as needed

        # Calculate row and cell strides
        grid_msg.row_stride = width * 5  # 5 bytes per cell if using RGBA + value
        grid_msg.cell_stride = 5  # Each cell has 5 bytes: 1 for value, 4 for RGBA

        # Define fields
        grid_msg.fields = [
            PackedElementField(name="value", offset=0, type=PackedElementField.UINT8),
            PackedElementField(name="red", offset=1, type=PackedElementField.UINT8),
            PackedElementField(name="green", offset=2, type=PackedElementField.UINT8),
            PackedElementField(name="blue", offset=3, type=PackedElementField.UINT8),
            PackedElementField(name="alpha", offset=4, type=PackedElementField.UINT8)
        ]

        # Prepare data
        data_array = np.array(msg.data)
        formatted_array = np.array2string(data_array, precision=2, separator=',', suppress_small=True)

        self.get_logger().info(formatted_array)
        # min_value = np.min(data_array[data_array >= 0])  # Exclude unknowns (-1)
        # max_value = np.max(data_array)
        min_value = self.stiffness_range[0]
        max_value = self.stiffness_range[1]

        # Scale data to 0–255 and convert to uint8
        normalized_data = np.clip(((data_array - min_value) / (max_value - min_value)) * 255, 0, 255).astype(np.uint8)
        normalized_data[data_array == -1] = 0  # Set unknowns to 0
        # print(normalized_data)
        formatted_array = np.array2string(normalized_data, precision=2, separator=',', suppress_small=True)

        self.get_logger().info(formatted_array)


        # Initialize grid data buffer
        grid_data = []
        for index, value in enumerate(normalized_data):
            # Assign color based on normalized value (example color scheme)
            red = int(value)
            green = 25
            blue = int(255 - value)  # Static blue for visualization
            alpha = self.uncertainty_alpha if uncertainty[index] > self.uncertainty_threshold else 255  # Fully opaque or transparent for unknown
            # alpha = 255
            # Append values for each cell: [value, red, green, blue, alpha]
            grid_data.extend([int(value), red, green, blue, alpha])

        # Populate the Grid message with data
        grid_msg.data = grid_data
        
        # Publish the Grid message
        self.terrain_map_publisher.publish(grid_msg)
        # Create the color bar image (height=50, width=256)
        height, width = 80, 1024
        color_bar = np.zeros((height, width, 3), dtype=np.uint8)

        for value in range(width):
            red = int(value/width * 255)
            green = 25
            blue = int((255 - value/width) * 255)
            color_bar[:, value] = (blue, green, red)  # OpenCV uses BGR format

        # Overlay value labels
        
        for value in range(min_value, max_value, int((max_value-min_value)/5)):  # Show labels every 25 units
            x_position = int((value - min_value) / (max_value - min_value) * width)
            cv2.putText(
                color_bar,
                str(value),
                (x_position, height-5),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),  # White text
                2,
                cv2.LINE_AA
            )

        # Convert the OpenCV image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(color_bar, encoding='bgr8')

        # Publish the Image message
        self.colorbar_publisher.publish(image_msg)




    def spatial_points_callback(self, msg: MeasurementArray):
        """Converts MeasurementArray data to MarkerArray for visualization."""
        trajectory_markers = MarkerArray()
        
        marker_id = 0
        for measurement in msg.measurements:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = (measurement.position.x - self.x_range[0])/(self.x_range[1] - self.x_range[0]) * self.resolution[1]
            marker.pose.position.y = (1.54 - measurement.position.z - self.y_range[0])/(self.y_range[1] - self.y_range[0]) * self.resolution[0]
            marker.pose.position.z = 0.0
            
            # Scale and color based on measurement value
            marker.scale.x = 9.0
            marker.scale.y = 9.0
            marker.scale.z = 9.0
            marker.color = self.get_color_by_spatial(measurement.value)
            trajectory_markers.markers.append(marker)
            marker_id += 1
        
        # Publish trajectory buffer size
        self.measurements_publisher.publish(trajectory_markers)

    def get_color_by_spatial(self, spatial):
        """Maps spatial value to a color for visualization."""
        color = ColorRGBA()
        color.r = max(0.0, min(1.0, spatial / 2000.0))
        color.g = 0.25

        
        color.b = max(0.0, min(1.0, 1.0 - spatial / 2000.0))
        color.r = 0.0
        color.g = 0.0
        color.b = 0.0
        color.a = 1.0  # Fully opaque
        return color

def main(args=None):
    rclpy.init(args=args)
    foxglove_node = Foxglove()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(foxglove_node)
    executor.spin()
    
    foxglove_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

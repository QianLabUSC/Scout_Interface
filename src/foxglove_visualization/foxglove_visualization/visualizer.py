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
from trusses_custom_interfaces.msg import ExtrapolatedMap, MeasurementArray
from std_msgs.msg import Header
from foxglove_msgs.msg import Grid, PackedElementField, Vector2
from geometry_msgs.msg import Pose
class Foxglove(Node):
    def __init__(self):
        super().__init__('foxglove_visualization_inter_')
        
        # QoS profile with KEEP_LAST history
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
        )
        
        self.spatial_map_subscriber = self.create_subscription(ExtrapolatedMap, 'extrapolated_map', self.spatial_map_callback, self.qos_profile)
        self.spatial_points_subscriber = self.create_subscription(MeasurementArray, 'collected_measurements', self.spatial_points_callback, self.qos_profile)
        

        # Publishers for buffer size
        self.terrain_map_publisher = self.create_publisher(Grid, 'terrain_map', 10)
        self.measurements_publisher = self.create_publisher(MarkerArray, 'measurements_markers', 10)
        self.colorbar_publisher = self.create_publisher(Image, 'terrain_color_bar', 10)
        self.bridge = CvBridge()
        

        # Declare parameters with default values
        self.declare_parameter('x_range', (-1.5, 1.5))
        self.declare_parameter('y_range', (-3, 3))
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
            marker.pose.position.y = (measurement.position.y - self.y_range[0])/(self.y_range[1] - self.y_range[0]) * self.resolution[0]
            marker.pose.position.z = 0.0
            
            # Scale and color based on measurement value
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
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

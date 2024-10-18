import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Int32

from std_msgs.msg import Float32MultiArray
import random


from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'trajectory_data', self.listener_callback, 10)
        self.marker_array = MarkerArray()
        self.terrain_marker = self.create_terrain_marker()
        self.create_terrain()
        self.get_logger().info("MarkerPublisher node has been started.")

    def listener_callback(self, msg):
        x = msg.data[0]
        y = msg.data[1]
        stiffness = msg.data[2]

        self.create_trajectory_marker(x, y, stiffness)
        self.publisher_.publish(self.marker_array)
        self.get_logger().info(f"Published markers: {len(self.marker_array.markers)}")

    def create_terrain_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "terrain"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0  # Alpha set to fully opaque
        self.get_logger().info("Created initial terrain marker.")
        return marker

    def create_terrain(self):
        size = 20  # Size of the grid
        step = 1.0  # Step size between points
        for i in range(size):
            for j in range(size):
                x1, y1 = i * step, j * step
                x2, y2 = (i + 1) * step, j * step
                x3, y3 = i * step, (j + 1) * step
                x4, y4 = (i + 1) * step, (j + 1) * step

                z1 = self.get_height(x1, y1)
                z2 = self.get_height(x2, y2)
                z3 = self.get_height(x3, y3)
                z4 = self.get_height(x4, y4)

                p1 = Point(x=x1, y=y1, z=z1)
                p2 = Point(x=x2, y=y2, z=z2)
                p3 = Point(x=x3, y=y3, z=z3)
                p4 = Point(x=x4, y=y4, z=z4)

                c1 = self.get_color_by_height(z1)
                c2 = self.get_color_by_height(z2)
                c3 = self.get_color_by_height(z3)
                c4 = self.get_color_by_height(z4)

                self.add_triangle(self.terrain_marker, p1, p2, p3, c1, c2, c3)
                self.add_triangle(self.terrain_marker, p2, p4, p3, c2, c4, c3)

        self.marker_array.markers.append(self.terrain_marker)

    def get_height(self, x, y):
        return (x * x + y * y) / 200.0  # Example function for terrain height

    def get_color_by_height(self, height):
        color = ColorRGBA()
        color.r = height / 10.0  # Normalize height for color
        color.g = 1.0 - height / 10.0
        color.b = 0.0
        color.a = 1.0
        return color

    def add_triangle(self, marker, p1, p2, p3, c1, c2, c3):
        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.colors.append(c1)
        marker.colors.append(c2)
        marker.colors.append(c3)

    def create_trajectory_marker(self, x, y, stiffness):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = len(self.marker_array.markers)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0 - stiffness
        marker.color.g = stiffness
        marker.color.b = 0.0

        point = Point()
        point.x = x
        point.y = y
        point.z = self.get_height(x, y)  # Place the point on the terrain

        if len(self.marker_array.markers) > 1 and self.marker_array.markers[-1].ns == "trajectory":
            self.marker_array.markers[-1].points.append(point)
        else:
            marker.points.append(point)
            self.marker_array.markers.append(marker)


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'trajectory_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [random.uniform(0.0, 10.0), random.uniform(0.0, 10.0), random.uniform(0.0, 1.0)]
        self.publisher_.publish(msg)




class TerrainNode(Node):
    def __init__(self):
        super().__init__('terrain_node')
        
        # QoS profile with KEEP_LAST history
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
        )
        
        # Publishers and subscribers
        self.publisher = self.create_publisher(MarkerArray, 'terrain_markers', self.qos_profile)
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.terrain_subscriber = self.create_subscription(MarkerArray, 'fake_terrain', self.terrain_callback, self.qos_profile)
        self.trajectory_subscriber = self.create_subscription(MarkerArray, 'fake_trajectory', self.trajectory_callback, self.qos_profile)
        
        self.terrain_markers = MarkerArray()
        self.trajectory_markers = MarkerArray()

        # Publishers for buffer size
        self.terrain_buffer_publisher = self.create_publisher(Int32, 'terrain_buffer_size', 10)
        self.trajectory_buffer_publisher = self.create_publisher(Int32, 'trajectory_buffer_size', 10)

    def terrain_callback(self, msg):
        self.terrain_markers = msg
        # Log the size of the terrain markers
        self.terrain_buffer_publisher.publish(Int32(data=len(self.terrain_markers.markers)))
    
    def trajectory_callback(self, msg):
        self.trajectory_markers = msg
        # Log the size of the trajectory markers
        self.trajectory_buffer_publisher.publish(Int32(data=len(self.trajectory_markers.markers)))

    def publish_markers(self):
        self.publisher.publish(self.terrain_markers)
        self.publisher.publish(self.trajectory_markers)

class FakeDataPublisher(Node):
    def __init__(self):
        super().__init__('fake_data_publisher')
        
        # Declare parameters
        self.declare_parameter('example_param', 'default_value')
        
        # QoS profile with KEEP_LAST history
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
        )
        
        self.terrain_publisher = self.create_publisher(MarkerArray, 'fake_terrain', self.qos_profile)
        self.trajectory_publisher = self.create_publisher(MarkerArray, 'fake_trajectory', self.qos_profile)
        self.timer = self.create_timer(0.1, self.publish_fake_data)
        self.grid_size = 100.0
        self.growth_rate = 0.1

    def interpolate_stiffness(self, resolution=0.1):
        x = np.arange(0, int(self.grid_size), dtype=float)
        y = np.arange(0, int(self.grid_size), dtype=float)
        xx, yy = np.meshgrid(x, y)
        zz = np.random.rand(int(self.grid_size), int(self.grid_size)).astype(float)
        x_new = np.arange(0, self.grid_size, resolution, dtype=float)
        y_new = np.arange(0, self.grid_size, resolution, dtype=float)
        xx_new, yy_new = np.meshgrid(x_new, y_new)
        zz_new = np.interp(xx_new.flatten(), x, zz[:,0]) + np.interp(yy_new.flatten(), y, zz[0,:])
        zz_new = zz_new.reshape(xx_new.shape)
        return xx_new, yy_new, zz_new

    def generate_trajectory(self, xx, yy, zz):
        trajectory_x = []
        trajectory_y = []
        trajectory_z = []
        for i in range(xx.shape[0]):
            if i % 2 == 0:
                for j in range(xx.shape[1]):
                    trajectory_x.append(xx[i, j])
                    trajectory_y.append(yy[i, j])
                    trajectory_z.append(zz[i, j])
            else:
                for j in reversed(range(xx.shape[1])):
                    trajectory_x.append(xx[i, j])
                    trajectory_y.append(yy[i, j])
                    trajectory_z.append(zz[i, j])
        return np.array(trajectory_x), np.array(trajectory_y), np.array(trajectory_z)

    def publish_fake_data(self):
        marker_array = MarkerArray()
        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()
        xx, yy, zz = self.interpolate_stiffness()
        marker_id = 0
        for i in range(xx.shape[0]):
            for j in range(xx.shape[1]):
                marker = Marker()
                marker.header = header
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(xx[i, j])
                marker.pose.position.y = float(yy[i, j])
                marker.pose.position.z = float(0.0)
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = float(zz[i, j])
                marker.color.a = 1.0
                marker.color.r = 1.0 - float(zz[i, j])
                marker.color.g = float(zz[i, j])
                marker.color.b = 0.0
                marker_array.markers.append(marker)
                marker_id += 1
        self.terrain_publisher.publish(marker_array)
        trajectory_x, trajectory_y, trajectory_z = self.generate_trajectory(xx, yy, zz)
        trajectory_marker_array = MarkerArray()
        for i in range(len(trajectory_x)):
            marker = Marker()
            marker.header = header
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(trajectory_x[i])
            marker.pose.position.y = float(trajectory_y[i])
            marker.pose.position.z = float(trajectory_z[i])
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            trajectory_marker_array.markers.append(marker)
            marker_id += 1
        self.trajectory_publisher.publish(trajectory_marker_array)
        self.grid_size += self.growth_rate

def main(args=None):
    rclpy.init(args=args)
    terrain_node = TerrainNode()
    fake_data_publisher = FakeDataPublisher()
    marker_publisher = MarkerPublisher()
    robot_publisher = TrajectoryPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(terrain_node)
    executor.add_node(fake_data_publisher)
    executor.add_node(marker_publisher)
    executor.add_node(robot_publisher)
    executor.spin()

    terrain_node.destroy_node()
    fake_data_publisher.destroy_node()
    robot_publisher.destroy_node()
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32
from trusses_custom_interfaces.msg import RealtimeMeasurement, RobotMeasurements
from trusses_custom_interfaces.msg import SpatialMeasurement, ExtrapolatedMap, MeasurementArray

class FakeDataPublisher(Node):
    def __init__(self):
        super().__init__('fake_data_publisher')
        
        self.gradient_rate = 0.05
        # Publishers for each message type on the specified topics
        self.spatial_measurement_publisher = self.create_publisher(SpatialMeasurement, 'spatial_measurements', 10)
        self.raw_measurments_publisher = self.create_publisher(RobotMeasurements, 'raw_measurements', 10)
        # self.extrapolated_map_publisher = self.create_publisher(ExtrapolatedMap, 'extrapolated_map', 10)
        # self.measurement_array_publisher = self.create_publisher(MeasurementArray, 'collected_measurements', 10)
        
        # Timers to publish each message type
        self.spatial_measurement_timer  = self.create_timer(1, self.publish_spatial_measurement)
        self.raw_measurement_timer  = self.create_timer(1, self.publish_raw_measurement)
        # self.extrapolated_map_timer  = self.create_timer(1, self.publish_extrapolated_map)
        # self.measurement_array_timer  = self.create_timer(1, self.publish_measurement_array)
    
        self.height = 100
        self.width = 50
        self.previous_data = 1000*np.ones(self.height * self.width)

        # Initialize measurement variables
        self.curr_pene = 0  # Boolean indicating penetration state
        self.pene_time = 0.0
        self.pene_depth = 0.0
        self.pene_force = 0.0
        self.time_step = 0.02  # Time increment for 50 Hz

        # Variables for simulation
        self.depth_increment = 0.1 / (3 / self.time_step)  # Increment per cycle step
        self.force_increment = 100 / (3 / self.time_step)  # Increment per cycle step
        self.state_duration = int(3 / self.time_step)  # Steps per state duration
        self.step_counter = 0

    def publish_raw_measurement(self):
        msg = RobotMeasurements()
        # Update penetration state
        self.step_counter += 1
        if self.step_counter < self.state_duration:
            self.curr_pene = 0
        elif self.step_counter < 2 * self.state_duration:
            self.curr_pene = 1
        else:
            self.step_counter = 0
            self.pene_depth = 0
            self.pene_force = 0

        # Update other measurements
        if self.curr_pene == 1:
            self.pene_time += self.time_step
            self.pene_depth += self.depth_increment
            self.pene_force += self.force_increment
            # Cap values at their maximum
            self.pene_time = min(self.pene_time, 3.0)
            self.pene_depth = min(self.pene_depth, 0.1)
            self.pene_force = min(self.pene_force, 100.0)
             # Populate and publish message
            msg.front_left_leg.curr_pene = bool(self.curr_pene)
            msg.front_left_leg.pene_time = self.pene_time
            msg.front_left_leg.pene_depth = self.pene_depth
            msg.front_left_leg.pene_force = self.pene_force
            msg.front_right_leg.curr_pene = bool(self.curr_pene)
            msg.front_right_leg.pene_time = self.pene_time  
            msg.front_right_leg.pene_depth = self.pene_depth + 0.05
            msg.front_right_leg.pene_force = self.pene_force + 50


            self.raw_measurments_publisher.publish(msg)

       


    def publish_spatial_measurement(self):
        msg = SpatialMeasurement()
        msg.position = Point32(x=np.random.uniform(-1.5, 1.5), y=np.random.uniform(-3, 3), z=np.random.uniform(0, 5))
        msg.value = np.random.uniform(0, 2000)
        msg.uncertainty = np.random.uniform(0, 10)
        msg.unit = "m"
        msg.source_name = "RandomSensor"
        msg.time = self.get_clock().now().to_msg()

        self.spatial_measurement_publisher.publish(msg)
        
        self.get_logger().info(f"Published SpatialMeasurement on 'spatial_measurement': position=({msg.position.x}, {msg.position.y}, {msg.position.z}), value={msg.value}")

    def generate_gradual_data(self):
        # Convert previous data to a numpy array for vectorized operations
        data = np.array(self.previous_data).reshape((self.height, self.width))

        # Define a small change rate for gradual changes over time
        self.gradient_rate += 0.01
        change_rate = np.max([1, self.gradient_rate])  # Adjust this for faster or slower temporal changes

        # Generate a spatial gradient (smooth change across rows and columns)
        x_gradient = np.linspace(0, 2000, self.width)  # Horizontal gradient
        y_gradient = np.linspace(0, 2000, self.height).reshape(-1, 1)  # Vertical gradient
        spatial_gradient = (x_gradient + y_gradient) / 2  # Combine for a smooth 2D gradient

        # Generate random small increments/decrements for temporal variation
        temporal_variation = np.random.uniform(-100, 100, (self.height, self.width)) * 10

        # Apply spatial gradient and add small temporal changes to previous data
        # new_data = data * (1-change_rate) + spatial_gradient * change_rate + temporal_variation
        new_data = spatial_gradient
        # Clip data to ensure it remains within the range [0, 100]
        new_data = np.clip(new_data, 0, 2000)
        uncertainty = 0.5 * new_data + 20 + np.random.uniform(0, 5, (self.height, self.width))

        return new_data.flatten().tolist(), uncertainty.flatten().tolist()  # Return as a 1D list to match the expected format


    def publish_extrapolated_map(self):
        msg = ExtrapolatedMap()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.meta.resolution = 1.0
        msg.meta.width = self.width
        msg.meta.height = self.height
        msg.height = self.height
        msg.width = self.width

        # Generate random data for the map
        new_data, uncertainty = self.generate_gradual_data()

        # Assign the generated data to the message
        msg.data = new_data
        # print(msg.data)
        msg.uncertainty = uncertainty

        self.extrapolated_map_publisher.publish(msg)
        self.get_logger().info(f"Published ExtrapolatedMap on 'spatial_map' with {self.height}x{self.width} grid.")

    def publish_measurement_array(self):
        msg = MeasurementArray()
        
        # Generate multiple SpatialMeasurement objects with random data
        num_measurements = 5
        for _ in range(num_measurements):
            measurement = SpatialMeasurement()
            measurement.position = Point32(x=np.random.uniform(-1.5, 1.5), y=np.random.uniform(-3, 3), z=np.random.uniform(0, 5))
            measurement.value = np.random.uniform(0, 100)
            measurement.uncertainty = np.random.uniform(0, 10)
            measurement.unit = "m"
            measurement.source_name = "RandomArraySensor"
            measurement.time = self.get_clock().now().to_msg()

            msg.measurements.append(measurement)

        # self.measurement_array_publisher.publish(msg)
        # self.get_logger().info(f"Published MeasurementArray on 'spatial_measurements' with {num_measurements} measurements.")

def main(args=None):
    rclpy.init(args=args)
    fake_data_publisher = FakeDataPublisher()
    rclpy.spin(fake_data_publisher)
    fake_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

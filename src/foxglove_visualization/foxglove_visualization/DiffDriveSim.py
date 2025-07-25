#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class DifferentialDriveSim(Node):
    def __init__(self):
        super().__init__('differential_drive_sim')

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading angle (rad)

        # Velocity commands
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'spirit/ghost_trot_control',
            self.cmd_vel_callback,
            10
        )

        # Publisher for pose (geometry_msgs/Pose)
        self.pose_pub = self.create_publisher(Pose, 'pose', 10)

        # TF broadcaster for map -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for simulation updates (10 Hz)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.update_pose)

        self.get_logger().info("Differential Drive Simulator with Pose & TF Started")

    def cmd_vel_callback(self, msg: Twist):
        """Callback to receive velocity commands."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_pose(self):
        """Update the robot pose using simple kinematics."""
        dt = self.timer_period

        # Differential drive model (2D)
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish the pose
        pose_msg = Pose()
        pose_msg.position = Point(x=self.x, y=self.y, z=0.0)

        # Quaternion from yaw (theta)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self.pose_pub.publish(pose_msg)

        # Publish TF map -> base_link
        self.publish_tf(qz, qw)

    def publish_tf(self, qz, qw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Differential Drive Simulator')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


import rclpy
from sensor_msgs.msg import Image
from scientific_payloads.camera_subscriber import CameraSubscriber


def camera_1_rgb_main(args=None):
    """Main function for front camera RGB subscriber"""
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(
        camera_number=1,
        node_name='camera_1_rgb_subscriber',
        topic='camera_1_image_topic',
        depth_alpha=0.03
    )
    
    # Override the source topic to match the specific topic name
    camera_subscriber.rgb_subscriber = camera_subscriber.create_subscription(
        Image,
        'camera_1_image_topic_rgb',
        camera_subscriber.rgb_callback,
        camera_subscriber.queue_size
    )
    
    try:
        camera_subscriber.get_logger().info('Front camera RGB subscriber started')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info('Front camera RGB subscriber shutting down')
    finally:
        camera_subscriber.cleanup()
        rclpy.shutdown()


def camera_1_depth_main(args=None):
    """Main function for front camera depth subscriber"""
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(
        camera_number=1,
        node_name='camera_1_depth_subscriber',
        topic='camera_1_image_topic',
        depth_alpha=0.03
    )
    
    # Override the source topic to match the specific topic name
    camera_subscriber.depth_subscriber = camera_subscriber.create_subscription(
        Image,
        'camera_1_image_topic_depth',
        camera_subscriber.depth_callback,
        camera_subscriber.queue_size
    )
    
    try:
        camera_subscriber.get_logger().info('Front camera depth subscriber started')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info('Front camera depth subscriber shutting down')
    finally:
        camera_subscriber.cleanup()
        rclpy.shutdown()


def camera_2_rgb_main(args=None):
    """Main function for back camera RGB subscriber"""
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(
        camera_number=2,
        node_name='camera_2_rgb_subscriber',
        topic='camera_2_image_topic',
        depth_alpha=0.03
    )
    
    # Override the source topic to match the specific topic name
    camera_subscriber.rgb_subscriber = camera_subscriber.create_subscription(
        Image,
        'camera_2_image_topic_rgb',
        camera_subscriber.rgb_callback,
        camera_subscriber.queue_size
    )
    
    try:
        camera_subscriber.get_logger().info('Back camera RGB subscriber started')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info('Back camera RGB subscriber shutting down')
    finally:
        camera_subscriber.cleanup()
        rclpy.shutdown()


def camera_2_depth_main(args=None):
    """Main function for back camera depth subscriber"""
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(
        camera_number=2,
        node_name='camera_2_depth_subscriber',
        topic='camera_2_image_topic',
        depth_alpha=0.03
    )
    
    # Override the source topic to match the specific topic name
    camera_subscriber.depth_subscriber = camera_subscriber.create_subscription(
        Image,
        'camera_2_image_topic_depth',
        camera_subscriber.depth_callback,
        camera_subscriber.queue_size
    )
    
    try:
        camera_subscriber.get_logger().info('Back camera depth subscriber started')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info('Back camera depth subscriber shutting down')
    finally:
        camera_subscriber.cleanup()
        rclpy.shutdown()


def navigation_camera_rgb_main(args=None):
    """Main function for navigation camera RGB subscriber"""
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(
        camera_number=3,
        node_name='navigation_camera_rgb_subscriber',
        topic='navigation_camera',
        depth_alpha=30.0  # Different alpha for navigation camera
    )
    
    # Override the source topic to match the specific topic name
    camera_subscriber.rgb_subscriber = camera_subscriber.create_subscription(
        Image,
        'navigation_camera_rgb',
        camera_subscriber.rgb_callback,
        camera_subscriber.queue_size
    )
    
    try:
        camera_subscriber.get_logger().info('Navigation camera RGB subscriber started')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info('Navigation camera RGB subscriber shutting down')
    finally:
        camera_subscriber.cleanup()
        rclpy.shutdown()


def navigation_camera_depth_main(args=None):
    """Main function for navigation camera depth subscriber"""
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(
        camera_number=3,
        node_name='navigation_camera_depth_subscriber',
        topic='navigation_camera',
        depth_alpha=30.0  # Different alpha for navigation camera
    )
    
    # Override the source topic to match the specific topic name
    camera_subscriber.depth_subscriber = camera_subscriber.create_subscription(
        Image,
        'navigation_camera_depth',
        camera_subscriber.depth_callback,
        camera_subscriber.queue_size
    )
    
    try:
        camera_subscriber.get_logger().info('Navigation camera depth subscriber started')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        camera_subscriber.get_logger().info('Navigation camera depth subscriber shutting down')
    finally:
        camera_subscriber.cleanup()
        rclpy.shutdown()
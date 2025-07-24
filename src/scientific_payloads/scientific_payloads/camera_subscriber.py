import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import imutils


class CameraSubscriber(Node):
    
    def __init__(self, CID=None, node_name='camera_subscriber_node', topic=None, camera_number=1, depth_alpha=0.03):
        super().__init__(node_name)
        
        self.camera_number = camera_number
        self.node_name = node_name
        self.depth_alpha = depth_alpha  # Alpha value for depth image scaling
        
        self.bridge = CvBridge()  # Create the bridge (translate from OpenCV to ROS)
        
        # Set the camera device
        if CID is None:
            self.camera_device_num = f"/dev/video{camera_number}"
        else:
            self.camera_device_num = CID
        
        # Set the topic base name
        if topic is None:
            self.topic_name_frames = f'camera_{camera_number}/image'
        else:
            self.topic_name_frames = topic
        
        self.queue_size = 20
        
        # Create publishers for Foxglove visualization
        # Raw image topics for Foxglove
        self.rgb_topic = f"{self.topic_name_frames}/rgb"
        self.depth_topic = f"{self.topic_name_frames}/depth"
        
        # Compressed image topics for better performance in Foxglove
        self.rgb_compressed_topic = f"{self.topic_name_frames}/rgb/compressed"
        self.depth_compressed_topic = f"{self.topic_name_frames}/depth/compressed"
        
        # Publishers for Foxglove visualization
        self.rgb_publisher = self.create_publisher(Image, self.rgb_topic, self.queue_size)
        self.depth_publisher = self.create_publisher(Image, self.depth_topic, self.queue_size)
        
        # Compressed publishers for better network performance
        self.rgb_compressed_publisher = self.create_publisher(
            CompressedImage, self.rgb_compressed_topic, self.queue_size
        )
        self.depth_compressed_publisher = self.create_publisher(
            CompressedImage, self.depth_compressed_topic, self.queue_size
        )
        
        # Initialize subscriber placeholders (will be overridden in main functions)
        self.rgb_subscriber = None
        self.depth_subscriber = None
        
        self.get_logger().info(f'Camera {camera_number} subscriber initialized')
        self.get_logger().info(f'Publishing RGB to: {self.rgb_topic}')
        self.get_logger().info(f'Publishing Depth to: {self.depth_topic}')
    
    def rgb_callback(self, image_msg):
        """Process RGB image and publish for Foxglove visualization"""
        try:
            # Republish the image for Foxglove
            self.rgb_publisher.publish(image_msg)
            
            # Create compressed version for better performance
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Resize for better performance if needed
            resized_image = imutils.resize(cv_image, width=640)
            
            # Create compressed message
            compressed_msg = CompressedImage()
            compressed_msg.header = image_msg.header
            compressed_msg.format = "jpeg"
            
            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, buffer = cv2.imencode('.jpg', resized_image, encode_param)
            compressed_msg.data = buffer.tobytes()
            
            self.rgb_compressed_publisher.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')
    
    def depth_callback(self, image_msg):
        """Process depth image and publish for Foxglove visualization"""
        try:
            # Republish the depth image for Foxglove
            self.depth_publisher.publish(image_msg)
            
            # Create compressed colorized depth image for visualization
            cv_image = self.bridge.imgmsg_to_cv2(image_msg)
            
            # Resize for better performance
            resized_image = imutils.resize(cv_image, width=640)
            
            # Apply colormap for better visualization
            colorized_depth = cv2.applyColorMap(
                cv2.convertScaleAbs(resized_image, alpha=self.depth_alpha), 
                cv2.COLORMAP_JET
            )
            
            # Create compressed message
            compressed_msg = CompressedImage()
            compressed_msg.header = image_msg.header
            compressed_msg.format = "jpeg"
            
            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, buffer = cv2.imencode('.jpg', colorized_depth, encode_param)
            compressed_msg.data = buffer.tobytes()
            
            self.depth_compressed_publisher.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def get_topic_info(self):
        """Return topic information for Foxglove configuration"""
        return {
            'camera_number': self.camera_number,
            'rgb_topic': self.rgb_topic,
            'depth_topic': self.depth_topic,
            'rgb_compressed_topic': self.rgb_compressed_topic,
            'depth_compressed_topic': self.depth_compressed_topic
        }
    
    def cleanup(self):
        """Clean up resources"""
        try:
            self.destroy_subscription(self.rgb_subscriber)
            self.destroy_subscription(self.depth_subscriber)
            
            self.destroy_publisher(self.rgb_publisher)
            self.destroy_publisher(self.depth_publisher)
            self.destroy_publisher(self.rgb_compressed_publisher)
            self.destroy_publisher(self.depth_compressed_publisher)
            
            if hasattr(self, 'bridge'):
                del self.bridge
                
            self.get_logger().info('Camera subscriber cleaned up')
            
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')
from setuptools import setup, find_packages

package_name = 'scientific_payloads'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'opencv-python',
        'imutils',
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Scientific payloads package for camera subscribers with Foxglove visualization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Front camera (D405) entry points
            'camera_1_rgb_subscriber = scientific_payloads.camera_main:camera_1_rgb_main',
            'camera_1_depth_subscriber = scientific_payloads.camera_main:camera_1_depth_main',
            
            # Back camera (D405) entry points
            'camera_2_rgb_subscriber = scientific_payloads.camera_main:camera_2_rgb_main',
            'camera_2_depth_subscriber = scientific_payloads.camera_main:camera_2_depth_main',
            
            # Navigation camera entry points
            'navigation_camera_rgb_subscriber = scientific_payloads.camera_main:navigation_camera_rgb_main',
            'navigation_camera_depth_subscriber = scientific_payloads.camera_main:navigation_camera_depth_main',
        ],
    },
)
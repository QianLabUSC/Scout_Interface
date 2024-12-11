#Copyright © 2018 Naturalpoint
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# OptiTrack NatNet direct depacketization library for Python 3.x

# modified by Henry Liu in RoboLAND(University of Sourthern California)

import pickle
import socket
import struct
from threading import Thread

from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import rosidl_generator_py


start = 0
def trace( *args ):
    pass # print( "".join(map(str,args)) )

# Create structs for reading various object types to speed up parsing.
Vector3 = struct.Struct( '<fff' )
Quaternion = struct.Struct( '<ffff' )
FloatValue = struct.Struct( '<f' )
DoubleValue = struct.Struct( '<d' )

class NatNetClient:
    def __init__( self ):
        # Change this value to the IP address of your local network interface
        self.localIPAddress = "192.168.20.177"

        # NatNet Data channel     
        self.dataPort = 8000 

        self.ros_publisher = None

        self.bufferSize  = 1024 * 32


        

    def dataThreadFunction(self, socket):
        while True:
            # Block for input
            data, addr = socket.recvfrom(self.bufferSize ) # 32k byte buffer size
            if( len( data ) > 0 ):
            # Send information to any listener.
                rot_pos = pickle.loads(data)
                rot = rot_pos[1:5]
                pos = rot_pos[5:8]
            if self.ros_publisher is not None:
                # print(rot)
                # print(pos)
                self.ros_publisher.publish_motion_msg(rot, pos)
                
    def run( self ):
        # Create the data socket
        self.dataSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.dataSocket.bind((self.localIPAddress, self.dataPort))
        if( self.dataSocket is None ):
            print( "Could not open data channel" )
            exit
        # Create a separate thread for receiving data packets
        dataThread = Thread( target = self.dataThreadFunction, args = (self.dataSocket, ))
        dataThread.start()


class OptitrackNode(Node):

    def __init__(self):
        super().__init__('OptitrackNode')
        self.publisher_ = self.create_publisher(Pose, 'spirit/mocap', 10)
        self.robot_pose = Pose()
        

    def publish_motion_msg(self, rotation, position):   
        current = time.time()
        if current - start > 5:
            # print( "Received frame for rigid body", id )
            # np.array(rotation)
            rot = np.array(rotation)
            pos = np.array(position)
            # print(rot, pos)
            self.robot_pose.position.x = pos[0]
            self.robot_pose.position.y = pos[1]
            self.robot_pose.position.z = pos[2]
            self.robot_pose.orientation.x = rot[0]
            self.robot_pose.orientation.y = rot[1]
            self.robot_pose.orientation.z = rot[2]
            self.robot_pose.orientation.w = rot[3]
            self.publisher_.publish(self.robot_pose)


def main(args=None):
    rclpy.init(args=args)

    ROSOptitrackNode = OptitrackNode()
    # This will create a new NatNet client
    streamingClient = NatNetClient()
    streamingClient.ros_publisher = ROSOptitrackNode
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()
    

    rclpy.spin(ROSOptitrackNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ROSOptitrackNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


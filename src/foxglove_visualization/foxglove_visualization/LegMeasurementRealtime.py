from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from trusses_custom_interfaces.msg import SpiritState
from geometry_msgs.msg import Pose
from trusses_custom_interfaces.msg import RobotMeasurements, SpatialMeasurement
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

class RealtimeSubscriber(Node):
    def __init__(self):
        super().__init__('measurement_subscriber')
        self.subscription_state = self.create_subscription(
            SpiritState,
            'spirit/state',
            self.SpiritState_callback,
            10)
        self.subscription_state  # prevent unused variable warning
        self.subscription_mocap = self.create_subscription(
            Pose,
            'spirit/mocap',
            self.Pose_callback,
            10)
        self.subscription_mocap  # prevent unused variable warning
        self.realtime_publisher = self.create_publisher(
            RobotMeasurements,
            'raw_measurements',
            10)
        self.realtime_publisher  # prevent unused variable warning
        self.spatial_measurement_publisher = self.create_publisher(
            SpatialMeasurement,
            'spatial_measurements',
            10)
        self.spatial_measurement_publisher  # prevent unused variable warning

        # initialize hip positions in body frame
        # now suppose the MoCap gives the CoM location & body orientation
        # the following parameters need to be measured precisely
        self.CoM_Hip_B = np.array([[0.228, -0.228, 0.228, -0.228],
                                    [0.07, 0.07, -0.07, -0.07],
                                    [0.0, 0.0, 0.0, 0.0]])
        # robot state
        self.spirit_state = SpiritState()
        # mocap state
        self.R_WB = np.identity(3)
        self.CoM_pos = np.array([0.0, 0.0, 0.0])
        # front left leg and fron right leg index
        self.idx_fl = 0
        self.idx_fr = 2
        # penetration raw data
        self.curr_pene = False
        self.pene_leg_idx = -1
        self.pene_time_fl = 0.0
        self.pene_depth_fl = 0.0
        self.pene_force_fl = 0.0
        self.pene_time_fr = 0.0
        self.pene_depth_fr = 0.0
        self.pene_force_fr = 0.0
        # buffer to calculate the penetration measurement
        # now only consider the front left and front right legs
        self.pene_time_buffer = []
        self.pene_depth_buffer = []
        self.pene_force_buffer = []
        self.stiffness = 0.0
        # plot configuration
        # self.lastframe_time = time.time()
        # self.plot_refresh_rate = 10
        '''
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.xlim_low = 0.0
        self.xlim_high = 0.0
        self.ylim_low = 0.0
        self.ylim_high = 0.0
        self.zlim_low = 0.0
        self.zlim_high = 0.0
        '''

    def forwardKinematicsSolver(self, jointVec,legNum):
        L_Upper=0.206
        L_Lower=0.206
        if legNum==0 or legNum==1:
            L_Adjust=0.10098
        else:
            L_Adjust=-0.10098

        theta_ab=jointVec[2]
        theta_hip=jointVec[0]
        theta_knee=jointVec[1]

        #now for 2d plane calculations
        pInter_xPlane = -L_Upper*np.cos(theta_hip) #gets intermediate point p in x direction of the 2d plane
        pInter_zPlane = -L_Upper*np.sin(theta_hip) #gets the intermediate point p in the z direction of the 2d plane

        pToe_xPlane = pInter_xPlane+L_Lower*np.cos(theta_knee-theta_hip) #gets the toe point in the x direction of the 2d plane
        pToe_zPlane = pInter_zPlane-L_Lower*np.sin(theta_knee-theta_hip) #gets the toe point in the z direction of the 2d plane

        #now for the offsets to convert from plane to real world in translation
        L_OffsetY = L_Adjust*np.cos(theta_ab) #gets the offset in the y direction from the robot leg y 0 to the 0 of the plane
        L_OffsetZ = L_Adjust*np.sin(theta_ab) #gets the offset in the z direction from the robot leg z 0 to the 0 of the plane

        #now for the toe positions
        toeX = pToe_xPlane #the x coordinate in the plane is the same as the final leg x
        toeY = L_OffsetY - pToe_zPlane*np.sin(theta_ab)  # gets the toe y position
        toeZ = L_OffsetZ + pToe_zPlane*np.cos(theta_ab) #gets the toe z position

        toeVec = [toeX,toeY,toeZ]
        return toeVec

    def update_measurement(self):
        #gets the custom mode
        self.pene_time_fl = self.spirit_state.mainboard_t
        self.pene_time_fr = self.spirit_state.mainboard_t
        custom_mode = self.spirit_state.mode[1]
        #gets the ghost behavior mode
        ghost_behav_mode = self.spirit_state.behavior[1]
        #gets forces during walk, i guess we will develop the later
        # spirit_forces = self.spirit_state.joint_residuals
        #gets user inputs
        user_data = self.spirit_state.user_custom
        current_penetrate = user_data[0]
        science_toe_idx = user_data[1]
        pos_penetrate = user_data[2:5]
        force_penetrate = user_data[5:8]
        # spirit_forces = self.spirit_state.joint_residuals
        # first checks what state we are in
        # custom, mode = behavior
        # 1, 50331648
        # 2, 83886080
        # 3, 117440512
        # 4, 150994944
        if (custom_mode < 1e8) or (custom_mode > 1.3e8) or (current_penetrate != 1):
            #we are not in crawl mode
            #and we are in the penetrate, for some reason the custom_Mode is very
            #high for penetrate so we check this
            if self.curr_pene and ((self.pene_leg_idx == self.idx_fl) or (self.pene_leg_idx == self.idx_fr)):
                self.stiffness = self.stiffness_calculation()
                self.spatial_measurement_publish()
            self.pene_leg_idx = -1
            self.curr_pene = False
            self.pene_time_buffer = []
            self.pene_depth_buffer = []
            self.pene_force_buffer = []
        else:
            self.curr_pene = True
            if self.curr_pene and (science_toe_idx != self.pene_leg_idx) and ((self.pene_leg_idx == self.idx_fl) or (self.pene_leg_idx == self.idx_fr)):
                self.stiffness = self.stiffness_calculation()
                self.spatial_measurement_publish()
                self.pene_time_buffer = []
                self.pene_depth_buffer = []
                self.pene_force_buffer = []
            self.pene_leg_idx = int(science_toe_idx)
            if self.pene_leg_idx == self.idx_fl:
                # front left leg is in penetration
                self.pene_depth_fl = -pos_penetrate[2]   # cropped toe z
                self.pene_force_fl = force_penetrate[2]   # cropped toe current z
                # buffer to calculate stiffness later
                self.pene_time_buffer.append(self.pene_time_fl)
                self.pene_depth_buffer.append(self.pene_depth_fl)
                self.pene_force_buffer.append(self.pene_force_fl)
                # we also want to know the force from front right leg
                self.pene_depth_fr = float('nan')
                self.pene_force_fr = float('nan')
            elif self.pene_leg_idx == self.idx_fr:
                # front right leg is in penetration
                self.pene_depth_fr = -pos_penetrate[2]   # cropped toe z
                self.pene_force_fr = force_penetrate[2]   # cropped toe current z
                # buffer to calculate stiffness later
                self.pene_time_buffer.append(self.pene_time_fr)
                self.pene_depth_buffer.append(self.pene_depth_fr)
                self.pene_force_buffer.append(self.pene_force_fr)
                # we also want to know the force from front left leg
                self.pene_depth_fl = float('nan')
                self.pene_force_fl = float('nan')

# if we don't know toe position and have to calculate from joint position
    def update_toePos_W(self):
        # here we use joint positions to get the leg position in body frame
        # and then convert the leg position from body frame to world frame
    	# toe's position in body frame
    	# initialize the toe position in body frame
        Hip_Toe_B = np.zeros((3,4))
        jointPos = self.spirit_state.joint_position
        if len(jointPos)==12:
            # jointPos = [0hip, 0knee, 1hip, 1knee, 2hip, 2knee, 3hip, 3knee, 4hip, 4knee, 0ab, 1ab, 2ab, 3ab]
            # now the jointPos is a 1x12 list, let's make it into a np 3x4 array
            jointPos = np.array([[jointPos[0],jointPos[2],jointPos[4],jointPos[6]],
                                    [jointPos[1],jointPos[3],jointPos[5],jointPos[7]],
                                    [jointPos[8],jointPos[9],jointPos[10],jointPos[11]]])
            # joint position offset
            jointPos[0] = jointPos[0] + 0.1807
            jointPos[1] = jointPos[1] + 0.2325
            # translate the jointPos to the Toe position in body frame
            for i in range(4):
                Hip_Toe_B[:,i] = self.forwardKinematicsSolver(jointPos[:,i], i)
            # we want the hip and toe positions in world frame
            # expand CoMPos to a 3x4 array
            self.Hip_W = np.tile(self.CoM_pos[:, np.newaxis], (1, 4)) + self.R_WB @ self.CoM_Hip_B
            self.Toe_W = self.Hip_W + self.R_WB @ Hip_Toe_B
        else:
            print("Joint Pos recv error")
        # determine if we need to update plot
        # if time.time() - self.lastframe_time > 1.0 / self.plot_refresh_rate:
        #     self.plot_4toes()

    def stiffness_calculation(self):
        depth = np.array(self.pene_depth_buffer)
        force = np.array(self.pene_force_buffer)
        # force that we recognize as start penetration
        force_threshold = 5.0
        # we start searching from the zero height
        depth_zero_idx = np.argmax(depth > -0.02)
        for i in range(depth_zero_idx, len(depth)):
            if force[i] > force_threshold:
                start_pene_idx = i
                break
        # Perform linear fit
        coefficients = np.polyfit(depth[start_pene_idx:-1], force[start_pene_idx:-1], 1)
        slope, intercept = coefficients
        return slope

    def SpiritState_callback(self, msg):
        self.spirit_state = msg
        # update toe position
        self.update_toePos_W()
        self.update_measurement()
        self.realtime_measurement_publish()

    def Pose_callback(self, msg):
        mocap_q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.CoM_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        # quaternion to rotation matrix, this is rotation matrix from MoCap to World
        R_WMo = Rotation.from_quat(mocap_q)
        self.R_WB = R_WMo.as_matrix()
        # update toe position
        # self.update_toePos_W()

    def realtime_measurement_publish(self):
        msg = RobotMeasurements()
        msg.front_left_leg.position.x = self.Toe_W[0,self.idx_fl]
        msg.front_left_leg.position.y = self.Toe_W[1,self.idx_fl]
        msg.front_left_leg.position.z = self.Toe_W[2,self.idx_fl]
        msg.front_left_leg.curr_pene = (self.pene_leg_idx == self.idx_fl)
        msg.front_left_leg.pene_time = self.pene_time_fl
        msg.front_left_leg.pene_depth = self.pene_depth_fl
        msg.front_left_leg.pene_force = self.pene_force_fl
        msg.front_right_leg.position.x = self.Toe_W[0,self.idx_fr]
        msg.front_right_leg.position.y = self.Toe_W[1,self.idx_fr]
        msg.front_right_leg.position.z = self.Toe_W[2,self.idx_fr]
        msg.front_right_leg.curr_pene = (self.pene_leg_idx == self.idx_fr)
        msg.front_right_leg.pene_time = self.pene_time_fr
        msg.front_right_leg.pene_depth = self.pene_depth_fr
        msg.front_right_leg.pene_force = self.pene_force_fr
        self.realtime_publisher.publish(msg)
        print(msg)
        # if self.curr_pene:
        #     print(self.pene_leg_idx)
        #     print(msg.front_left_leg.position)
        #     print(msg.front_right_leg.position)
        # else:
        #     print("------------")

    def spatial_measurement_publish(self):
        msg = SpatialMeasurement()
        if (self.pene_leg_idx == self.idx_fl) or (self.pene_leg_idx == self.idx_fr):
            msg.position.x = self.Toe_W[0,self.pene_leg_idx]
            msg.position.y = self.Toe_W[0,self.pene_leg_idx]
            msg.position.z = self.Toe_W[0,self.pene_leg_idx]
        msg.uncertainty = 0.0
        msg.value = self.stiffness
        msg.unit = "N/m"
        msg.source_name = "Stiffness"
        msg.time = self.get_clock().now().to_msg()
        # print(self.pene_leg_idx)
        # print(msg)
        self.spatial_measurement_publisher.publish(msg)

    '''
    def plot_4toes(self):
        # use the toe position to plot and update the frame every time called
        self.ax.clear()
        self.ax.scatter(self.Toe_W[0,:], self.Toe_W[1,:], self.Toe_W[2,:])
        # Adjust the limit accordingly
        self.xlim_low = np.min([self.xlim_low, np.min(self.Toe_W[0,:])-1.0])
        self.xlim_high = np.max([self.xlim_high, np.max(self.Toe_W[0,:])+1.0])
        self.ylim_low = np.min([self.ylim_low, np.min(self.Toe_W[1,:])-1.0])
        self.ylim_high = np.max([self.ylim_high, np.max(self.Toe_W[1,:])+1.0])
        self.zlim_low = np.min([self.zlim_low, np.min(self.Toe_W[2,:])-1.0])
        self.zlim_high = np.max([self.zlim_high, np.max(self.Toe_W[2,:])+1.0])
        self.ax.set_xlim(self.xlim_low, self.xlim_high)
        self.ax.set_ylim(self.ylim_low, self.ylim_high)
        self.ax.set_zlim(self.zlim_low, self.zlim_high)
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        # self.ax.set_aspect('equal')
        plt.draw()
        plt.pause(1.0 / self.plot_refresh_rate)
'''

def main(args=None):
    rclpy.init(args=args)

    realtime_subscriber = RealtimeSubscriber()

    rclpy.spin(realtime_subscriber)

    realtime_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


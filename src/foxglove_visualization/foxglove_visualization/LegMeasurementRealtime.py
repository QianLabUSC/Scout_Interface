from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from trusses_custom_interfaces.msg import SpiritState
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from trusses_custom_interfaces.msg import RobotMeasurements, SpatialMeasurement
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

class RealtimeSubscriber(Node):
    def __init__(self):
        super().__init__('measurement_subscriber')
        
        # Declare parameters
        self.declare_parameter('enable_visualization', False)
        
        # Read parameters
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        self.subscription_state = self.create_subscription(
            SpiritState,
            'spirit/state_low_speed',
            self.SpiritState_callback,
            10)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_state = self.create_subscription(
            SpiritState,
            'spirit/state_high_speed',
            self.SpiritState_callback,
            qos_profile
        )  
        self.subscription_state  # prevent unused variable warning
        self.subscription_current_pose = self.create_subscription(
            Pose,
            'spirit/current_pose',
            self.Pose_callback,
            10)
        self.subscription_current_pose  # prevent unused variable warning
       
        
        self.realtime_publisher = self.create_publisher(
            RobotMeasurements,
            'spirit/raw_measurements',
            10)
        self.realtime_publisher  # prevent unused variable warning
        self.realtime_pene_publisher = self.create_publisher(
            RobotMeasurements,
            'spirit/raw_pene_measurements',
            10)
        self.realtime_pene_publisher  # prevent unused variable warning
        self.spatial_measurement_publisher = self.create_publisher(
            SpatialMeasurement,
            'spirit/spatial_measurements',
            10)
        self.spatial_measurement_publisher  # prevent unused variable warning
        
        
        # Only create visualization publishers if visualization is enabled
        if self.enable_visualization:
            self.robot_marker_publisher = self.create_publisher(
                Marker,
                'spirit/current_pose_marker',
                10)
            self.robot_marker_publisher  # prevent unused variable warning
            
            self.path_publisher = self.create_publisher(
                Path,
                'spirit/robot_path',
                10)
            self.path_publisher  # prevent unused variable warning
            
            # Initialize robot path
            self.robot_path = Path()
            self.robot_path.header.frame_id = "map"
            
            self.get_logger().info("Robot pose and path visualization enabled")
        else:
            self.get_logger().info("Robot pose and path visualization disabled")

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
        
        # buffers for trotting mode stiffness calculation
        self.trot_time_buffer_fl = []
        self.trot_depth_buffer_fl = []
        self.trot_force_buffer_fl = []
        self.trot_time_buffer_fr = []
        self.trot_depth_buffer_fr = []
        self.trot_force_buffer_fr = []
        
        # trotting mode state tracking
        self.prev_ghost_contact_fl = 1  # assume in contact initially
        self.prev_ghost_contact_fr = 1  # assume in contact initially
        self.recording_cycle_fl = False  # track if we're recording a cycle for FL
        self.recording_cycle_fr = False  # track if we're recording a cycle for FR
        
        # penetration detection for trotting mode
        self.prev_depth_fl = 0.0
        self.prev_depth_fr = 0.0
        self.prev_force_fl = 0.0
        self.prev_force_fr = 0.0
        self.penetration_detected_fl = False
        self.penetration_detected_fr = False
        
        self.stiffness = 0.0
        self.r2_crawl = 0.0  # R² for crawl mode stiffness fit
        self.r2_trot_fl = 0.0  # R² for trotting FL stiffness fit  
        self.r2_trot_fr = 0.0  # R² for trotting FR stiffness fit
        self.jointVec = np.zeros((3,4))
        self.jointCurrent = np.zeros((3,4))
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

    def forwardKinematicsSolver(self, jointVec, legNum):
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

    def jacobianSolver(self, jointVec, legNum):
        L_Upper=0.206
        L_Lower=0.206
        if legNum==0 or legNum==1:
            L_Adjust=-0.10098
        else:
            L_Adjust=0.10098
        # theta0 = theta_hip
        # theta1 = theta_knee
        # theta2 = theta_ab
        theta0=jointVec[0]
        theta1=jointVec[1]
        theta2=jointVec[2]
        sin0=np.sin(theta0)
        cos0=np.cos(theta0)
        sin1m0=np.sin(theta1-theta0)
        cos1m0=np.cos(theta1-theta0)
        sin2=np.sin(theta2)
        cos2=np.cos(theta2)
        z1=L_Upper*sin0+L_Lower*sin1m0
        dz1_theta0=L_Upper*cos0-L_Lower*cos1m0
        dz1_theta1=L_Lower*cos1m0
        J=np.zeros((3,3),dtype=np.float64)
        J[0,0]=L_Upper*sin0+L_Lower*sin1m0
        J[0,1]=-L_Lower*sin1m0
        J[1,0]=-dz1_theta0*sin2
        J[1,1]=-dz1_theta1*sin2
        J[1,2]=-L_Adjust*sin2-z1*cos2
        J[2,0]=-dz1_theta0*cos2
        J[2,1]=-dz1_theta1*cos2
        J[2,2]=-L_Adjust*cos2+z1*sin2
        return J

    def toeForceSolver(self, jointVec, jointCurrent, legNum):
        J = self.jacobianSolver(jointVec, legNum)
        try:
            JT_inv = np.linalg.inv(np.transpose(J))
        except:
            JT_inv = np.full((3,3),np.nan)
        jointTorque = 0.546*jointCurrent
        jointTorque[1] = 2.0*jointTorque[1]
        toeForce = JT_inv@jointTorque
        return toeForce

    def findPenetrationPortion(JointResidualZ):
        # prominence to be tuned
        # this prominence is tuned to find the penetration force peaks
        prominence0 = 50.0
        peaks0, properties0 = find_peaks(JointResidualZ, prominence=prominence0)
        # will only pick penetration force from 30 N to 60 N
        forcePeneLowerThreshold = 20
        forcePeneUpperThreshold = 60
        # find the penetration portion
        peneStartIdx = np.array([], dtype=int)
        peneEndIdx = np.array([], dtype=int)
        for i in range(1, len(peaks0)):
            currPeakIdx = peaks0[i]
            # find the valley between peaks0[i-1] and peaks0[i]
            currValleyIdx = np.argmin(JointResidualZ[peaks0[i - 1] : peaks0[i]])
            currValleyIdx = currValleyIdx + peaks0[i - 1]
            inPenetration = False
            peneStartIdxCurr = currValleyIdx + 1
            peneEndIdxCurr = currPeakIdx
            if JointResidualZ[currValleyIdx + 1] > forcePeneLowerThreshold:
                inPenetration = True
            for j in range(currValleyIdx + 1, currPeakIdx + 1):
                if inPenetration:
                    if JointResidualZ[j] > forcePeneUpperThreshold:
                        inPenetration = False
                        peneEndIdxCurr = j
                        break
                else:
                    if JointResidualZ[j - 1] < forcePeneLowerThreshold and JointResidualZ[j] > forcePeneLowerThreshold:
                        inPenetration = True
                        peneStartIdxCurr = j
            peneStartIdx = np.append(peneStartIdx, peneStartIdxCurr)
            peneEndIdx = np.append(peneEndIdx, peneEndIdxCurr)
        return peneStartIdx, peneEndIdx


    def update_measurement(self):
        #gets the custom mode
        try:
            custom_mode = self.spirit_state.mode[1]
        except Exception as e:
            self.get_logger().error("MODE NOT SET, not ready yet")
            return
        self.pene_time_fl = self.spirit_state.mainboard_t
        self.pene_time_fr = self.spirit_state.mainboard_t
        #gets the ghost behavior mode
        ghost_behav_mode = self.spirit_state.behavior[1]
        ghost_contact = self.spirit_state.contacts
        
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
        # first upadate forces calculated from Jacobian
        fl_toe_force = self.toeForceSolver(self.jointVec[:,self.idx_fl], self.jointCurrent[:,self.idx_fl], self.idx_fl)
        fr_toe_force = self.toeForceSolver(self.jointVec[:,self.idx_fr], self.jointCurrent[:,self.idx_fr], self.idx_fr)
        # note that this is in body frame, let's keep it for now for the demo
        self.pene_force_fl = -fl_toe_force[2]
        self.pene_force_fr = -fr_toe_force[2]





        print("ghost_behav_mode", ghost_behav_mode, "custom_mode", custom_mode)
        if (ghost_behav_mode  > 1 and ghost_behav_mode < 1e6): # I dont know why ghost behav mode is 524290, some decode not right. 
            # we are in ghost trotting mode 
            print("ghost trotting")
            current_ghost_contact_fl = ghost_contact[self.idx_fl]
            current_ghost_contact_fr = ghost_contact[self.idx_fr]
            
            # Get toe positions for depth calculation
            fl_toe_pos = self.forwardKinematicsSolver(self.jointVec[:,self.idx_fl], self.idx_fl)
            fr_toe_pos = self.forwardKinematicsSolver(self.jointVec[:,self.idx_fr], self.idx_fr)
            
            # Calculate raw depth (negative z position indicates penetration)
            depth_fl = -fl_toe_pos[2]
            depth_fr = -fr_toe_pos[2]
            
            # Get raw forces from joint residuals (format: [fz0, fx0, fz1, fx1, fz2, fx2, fz3, fx3])
            force_fl = self.spirit_state.joint_residuals[0]  # front left leg fz
            force_fr = self.spirit_state.joint_residuals[4]  # front right leg fz (index 4 = fz2)
            
            # Front Left Leg Processing
            # Check if leg left contact again (end of cycle) - CHECK THIS FIRST
            if self.prev_ghost_contact_fl == 1 and current_ghost_contact_fl == 0 and self.recording_cycle_fl:
                # Complete cycle captured - calculate stiffness directly from trotting buffers
                if len(self.trot_depth_buffer_fl) > 20:
                    # Calculate stiffness directly from trotting buffers
                    trot_stiffness_fl, r2_fl = self.stiffness_calculation_trotting(
                        self.trot_time_buffer_fl, 
                        self.trot_depth_buffer_fl, 
                        self.trot_force_buffer_fl
                    )
                    # Store R² for FL
                    self.r2_trot_fl = r2_fl
                    # Publish trotting measurement for FL
                    if r2_fl > 0.5:
                        self.publish_trotting_measurement(self.idx_fl, trot_stiffness_fl, r2_fl)
                    else:
                        print(f"Trotting FL stiffness R² too low: {r2_fl:.3f}, not publishing")
                else:
                    print("not enough data to calculate stiffness FL")
                
                self.recording_cycle_fl = False # End recording
            
            # Check if leg just left contact (start of cycle) - CHECK THIS SECOND
            elif self.prev_ghost_contact_fl == 1 and current_ghost_contact_fl == 0 and not self.recording_cycle_fl:
                # Leg just left contact - START recording new cycle, empty buffer
                self.trot_time_buffer_fl = []
                self.trot_depth_buffer_fl = []
                self.trot_force_buffer_fl = []
                self.recording_cycle_fl = True # Start recording
                self.penetration_detected_fl = False # Reset penetration detection
                self.prev_depth_fl = depth_fl  # Initialize previous values
                self.prev_force_fl = force_fl
            
            # Record data throughout the entire cycle (non-contact + contact phases)
            if self.recording_cycle_fl:
                self.trot_time_buffer_fl.append(self.spirit_state.mainboard_t)
                self.trot_depth_buffer_fl.append(depth_fl)
                self.trot_force_buffer_fl.append(force_fl)
                
                # Detect when depth starts to increase (penetration starts)
                if not self.penetration_detected_fl and len(self.trot_depth_buffer_fl) > 1:
                    force_threshold = 12.0  # Same threshold used in stiffness calculation
                    if depth_fl > self.prev_depth_fl and force_fl > self.prev_force_fl and force_fl > force_threshold:
                        # Depth is increasing, force is increasing and above threshold - meaningful penetration detected
                        self.penetration_detected_fl = True
                
                self.prev_depth_fl = depth_fl
                self.prev_force_fl = force_fl
            
            # Front Right Leg Processing
            # Check if leg left contact again (end of cycle) - CHECK THIS FIRST
            if self.prev_ghost_contact_fr == 1 and current_ghost_contact_fr == 0 and self.recording_cycle_fr:
                # Complete cycle captured - calculate stiffness directly from trotting buffers
                if len(self.trot_depth_buffer_fr) > 20:
                    # Calculate stiffness directly from trotting buffers
                    trot_stiffness_fr, r2_fr = self.stiffness_calculation_trotting(
                        self.trot_time_buffer_fr,
                        self.trot_depth_buffer_fr, 
                        self.trot_force_buffer_fr
                    )
                    # Store R² for FR
                    self.r2_trot_fr = r2_fr
                    # Publish trotting measurement for FR
                    if r2_fr > 0.5:
                        self.publish_trotting_measurement(self.idx_fr, trot_stiffness_fr, r2_fr)
                    else:
                        print(f"Trotting FR stiffness R² too low: {r2_fr:.3f}, not publishing")
                else:
                    print("not enough data to calculate stiffness")
                self.recording_cycle_fr = False # End recording
                
            # Check if leg just left contact (start of cycle) - CHECK THIS SECOND  
            elif self.prev_ghost_contact_fr == 1 and current_ghost_contact_fr == 0 and not self.recording_cycle_fr:
                # Leg just left contact - START recording new cycle, empty buffer
                self.trot_time_buffer_fr = []
                self.trot_depth_buffer_fr = []
                self.trot_force_buffer_fr = []
                self.recording_cycle_fr = True # Start recording
                self.penetration_detected_fr = False # Reset penetration detection
                self.prev_depth_fr = depth_fr  # Initialize previous values
                self.prev_force_fr = force_fr
            
            # Record data throughout the entire cycle (non-contact + contact phases)
            if self.recording_cycle_fr:
                self.trot_time_buffer_fr.append(self.spirit_state.mainboard_t)
                self.trot_depth_buffer_fr.append(depth_fr)
                self.trot_force_buffer_fr.append(force_fr)
                
                # Detect when depth starts to increase (penetration starts)
                if not self.penetration_detected_fr and len(self.trot_depth_buffer_fr) > 10:
                    force_threshold = 12.0  # Same threshold used in stiffness calculation
                    if depth_fr > self.prev_depth_fr and force_fr > self.prev_force_fr and force_fr > force_threshold:
                        # Depth is increasing, force is increasing and above threshold - meaningful penetration detected
                        self.penetration_detected_fr = True
                else:
                    print("not enough data to calculate stiffness")
                self.prev_depth_fr = depth_fr
                self.prev_force_fr = force_fr
            
            # Update previous contact states for next iteration
            self.prev_ghost_contact_fl = current_ghost_contact_fl
            self.prev_ghost_contact_fr = current_ghost_contact_fr
        elif (custom_mode < 1e8) or (custom_mode > 1.1e8) :
            print("ghost crawling")
            # we are not in ghost trotting mode. and we are in custom mode
            if (current_penetrate != 1):
                #and we are in the penetrate, for some reason the custom_Mode is very
                #high for penetrate so we check this
                if self.curr_pene:
                    # only calculate stiffness while on front right leg
                    if self.pene_leg_idx == self.idx_fr:
                        self.stiffness, self.r2_crawl = self.stiffness_calculation()
                        if self.r2_crawl > 0.5:
                            self.spatial_measurement_publish()
                        else:
                            print(f"Crawl stiffness R² too low: {self.r2_crawl:.3f}, not publishing")
                self.pene_leg_idx = -1
                self.curr_pene = False
                self.pene_time_buffer = []
                self.pene_depth_buffer = []
                self.pene_force_buffer = []
  
            else:
                # otherwise we are in crawl mode
                self.curr_pene = True
                if self.curr_pene and (science_toe_idx != self.pene_leg_idx):
                    # only calculate stiffness while on front right leg
                    if self.pene_leg_idx == self.idx_fr:
                        self.stiffness, self.r2_crawl = self.stiffness_calculation()
                        if self.r2_crawl > 0.5:
                            self.spatial_measurement_publish()
                        else:
                            print(f"Crawl stiffness R² too low: {self.r2_crawl:.3f}, not publishing")
                        self.pene_time_buffer = []
                        self.pene_depth_buffer = []
                        self.pene_force_buffer = []
                self.pene_leg_idx = int(science_toe_idx)
                # the depth of non penetration leg should be 0
                self.pene_depth_fl = 0.0
                self.pene_depth_fr = 0.0
                if self.pene_leg_idx == self.idx_fl:
                    # front left leg is in penetration
                    self.pene_depth_fl = -pos_penetrate[2]   # cropped toe z
                    self.pene_force_fl = force_penetrate[2]   # cropped toe current z
                    # buffer to calculate stiffness later
                    self.pene_time_buffer.append(self.pene_time_fl)
                    self.pene_depth_buffer.append(self.pene_depth_fl)
                    self.pene_force_buffer.append(self.pene_force_fl)
                elif self.pene_leg_idx == self.idx_fr:
                    # front right leg is in penetration
                    self.pene_depth_fr = -pos_penetrate[2]   # cropped toe z
                    self.pene_force_fr = force_penetrate[2]   # cropped toe current z
                    # buffer to calculate stiffness later
                    self.pene_time_buffer.append(self.pene_time_fr)
                    self.pene_depth_buffer.append(self.pene_depth_fr)
                    self.pene_force_buffer.append(self.pene_force_fr)
        else: # not in trotting or crawl mode
            # we should empty both buffers.
            # Clear crawl mode buffers
            self.pene_time_buffer = []
            self.pene_depth_buffer = []
            self.pene_force_buffer = []
            
            # Clear trotting mode buffers
            self.trot_time_buffer_fl = []
            self.trot_depth_buffer_fl = []
            self.trot_force_buffer_fl = []
            self.trot_time_buffer_fr = []
            self.trot_depth_buffer_fr = []
            self.trot_force_buffer_fr = []

    # if we don't know toe position and have to calculate from joint position
    def update_toePos_W(self):
        # here we use joint positions to get the leg position in body frame
        # and then convert the leg position from body frame to world frame
    	# toe's position in body frame
    	# initialize the toe position in body frame
        Hip_Toe_B = np.zeros((3,4))
        for i in range(4):
            Hip_Toe_B[:,i] = self.forwardKinematicsSolver(self.jointVec[:,i], i)
        # we want the hip and toe positions in world frame
        # expand CoMPos to a 3x4 array
        self.Hip_W = np.tile(self.CoM_pos[:, np.newaxis], (1, 4)) + self.R_WB @ self.CoM_Hip_B
        self.Toe_W = self.Hip_W + self.R_WB @ Hip_Toe_B
        # determine if we need to update plot
        # if time.time() - self.lastframe_time > 1.0 / self.plot_refresh_rate:
        #     self.plot_4toes()

    def stiffness_calculation(self):
        depth = np.array(self.pene_depth_buffer)
        force = np.array(self.pene_force_buffer)
        maxforce = np.max(force)
        # force that we recognize as start penetration
        # if the maxforce is small, we select the force threshold as 0.90*maxforce, this ensures we get data even if max penetration force is below 10
        force_threshold = min(12.0, 0.50*maxforce)
        # we start searching from the zero height
        # sometimes the depth always greater than 0.0, so we select -0.02
        # the function argmax will give index 0 if cannot find even one satisfying the condition
        depth_zero_idx = 0 #np.argmax(depth > -0.02)
        start_pene_idx = depth_zero_idx
        for i in range(depth_zero_idx, len(depth)):
            if force[i] > force_threshold:
                start_pene_idx = i
                break
        # Perform linear fit
        """
        WE MAY HAVE AN ISSUE WITH POLYFIT GOING TO END OF DATA. THIS IS BECAUSE
        WE HAVE A CLUMPING OF DATA AT THE END. WILL CAUSE SKEWING. FOR NOW LETS 
        DO START_PENE_IDX TO INDEX OF 0.95 OF MAX FORCE 
        
        """
        #FIND END PENE IDX
        max_force_threshold = 0.95*maxforce
        for i in range(depth_zero_idx,len(depth)):
            if force[i] < max_force_threshold:
                end_pene_idx = i
        
        # Ensure we have enough data points for linear fit
        if end_pene_idx - start_pene_idx < 2:
            return 0.0, 0.0  # Return stiffness=0, R²=0 if not enough data
            
        try:
            coefficients = np.polyfit(depth[start_pene_idx:end_pene_idx], force[start_pene_idx:end_pene_idx], 1)
            slope, intercept = coefficients
            
            # Calculate R² (coefficient of determination)
            y_actual = force[start_pene_idx:end_pene_idx]
            y_predicted = slope * depth[start_pene_idx:end_pene_idx] + intercept
            ss_res = np.sum((y_actual - y_predicted) ** 2)
            ss_tot = np.sum((y_actual - np.mean(y_actual)) ** 2)
            r2 = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0.0
            
            return slope, r2
        except:
            return 0.0, 0.0

    def stiffness_calculation_trotting(self, time_buffer, depth_buffer, force_buffer):
        """Calculate stiffness for trotting mode with data cropping"""
        # Ensure buffers are not empty
        if len(time_buffer) == 0 or len(depth_buffer) == 0 or len(force_buffer) == 0:
            return 0.0, 0.0 # Return stiffness=0, R²=0 if buffers are empty

        depth = np.array(depth_buffer)
        force = np.array(force_buffer)
        
        # Step 1: Find where depth starts to increase (leg starts to penetrate)
        # Look for the point where depth starts increasing from minimum
        depth_start_idx = 0
        min_depth = np.min(depth)
        min_depth_idx = np.argmin(depth)
        
        # Find where depth starts increasing after reaching minimum
        for i in range(min_depth_idx, len(depth)-1):
            if depth[i+1] > depth[i]:  # depth starts increasing
                depth_start_idx = i
                break
        
        # Step 2: Crop data from penetration start
        cropped_depth = depth[depth_start_idx:]
        cropped_force = force[depth_start_idx:]
        
        if len(cropped_depth) < 3:  # Not enough data points
            return 0.0, 0.0
            
        # Step 3: Find maximum force in cropped data
        maxforce = np.max(cropped_force)
        
        # Step 4: Use force threshold to find start penetration index
        # Force that we recognize as start penetration
        force_threshold = min(12.0, 0.50*maxforce)
        
        start_pene_idx = 0
        for i in range(len(cropped_force)):
            if cropped_force[i] > force_threshold:
                start_pene_idx = i
                break
                
        # Step 5: Find end penetration index (avoid data clumping at the end)
        max_force_threshold = 0.95*maxforce
        end_pene_idx = len(cropped_force) - 1
        for i in range(len(cropped_force)):
            if cropped_force[i] > max_force_threshold:
                end_pene_idx = i
                break
        
        # Ensure we have enough data points for linear fit
        if end_pene_idx - start_pene_idx < 2:
            return 0.0, 0.0
            
        try:
            # Perform linear fit on the penetration portion
            coefficients = np.polyfit(cropped_depth[start_pene_idx:end_pene_idx], 
                                    cropped_force[start_pene_idx:end_pene_idx], 1)
            slope, intercept = coefficients
            
            # Calculate R² (coefficient of determination)
            y_actual = cropped_force[start_pene_idx:end_pene_idx]
            y_predicted = slope * cropped_depth[start_pene_idx:end_pene_idx] + intercept
            ss_res = np.sum((y_actual - y_predicted) ** 2)
            ss_tot = np.sum((y_actual - np.mean(y_actual)) ** 2)
            r2 = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0.0
            
            return slope, r2
        except:
            return 0.0, 0.0

    def publish_trotting_measurement(self, leg_idx, stiffness, r2=0.0):
        msg = SpatialMeasurement()
        if (leg_idx == self.idx_fr) or (leg_idx == self.idx_fl):

            # transform_to_map_T_MW = np.array(
            #     [[-1, 0, 0, 0],
            #      [ 0,-1, 0, 2.4],
            #      [ 0, 0, 1, 0],
            #      [ 0, 0, 0, 1]]
            # )
            p_WT_homo = np.zeros((4,1))
            p_WT_homo[0:3,0] = self.Toe_W[:,leg_idx]
            # p_WT_homo[3,0]   = 1
            # p_MT_homo = transform_to_map_T_MW @ p_WT_homo

            msg.position.x = p_WT_homo[0,0]
            msg.position.y = p_WT_homo[1,0]
            msg.position.z = p_WT_homo[2,0]

        msg.uncertainty = r2  # Store R² as uncertainty measure
        msg.leg_idx = leg_idx
        msg.value = stiffness
        msg.unit = "N/m"
        msg.source_name = "Trotting"
        msg.time = self.get_clock().now().to_msg()
        # print(self.pene_leg_idx)
        # print(msg)
        self.spatial_measurement_publisher.publish(msg)

    def SpiritState_callback(self, msg):
        self.spirit_state = msg
        # self.get_logger().info("aaaaaaaaaaaaaaaaaa")
        # update self.jointVec
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
            self.jointVec = jointPos
        else:
            print("Joint Pos recv error")
        # debug for jointPos
        # print(jointPos)
        # update self.jointCurrent
        jointCurr = self.spirit_state.joint_currents
        if len(jointCurr)==12:
            # jointCurr = [0hip, 0knee, 1hip, 1knee, 2hip, 2knee, 3hip, 3knee, 4hip, 4knee, 0ab, 1ab, 2ab, 3ab]
            # now the jointCurr is a 1x12 list, let's make it into a np 3x4 array
            jointCurr = np.array([[jointCurr[0],jointCurr[2],jointCurr[4],jointCurr[6]],
                                    [jointCurr[1],jointCurr[3],jointCurr[5],jointCurr[7]],
                                    [jointCurr[8],jointCurr[9],jointCurr[10],jointCurr[11]]])
            self.jointCurrent = jointCurr
        else:
            print("Joint Curr recv error")
        # update toe position
        #gets forces during walk, i guess we will develop the later
        spirit_forces = self.spirit_state.joint_residuals #change 
        self.update_toePos_W()
        self.update_measurement()
        self.realtime_measurement_publish()

    def Pose_callback(self, msg):
        # Get data from mocap
        mocap_q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        p_WMo_W = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        
        # Init Rotations
        # quaternion to rotation matrix, this is rotation matrix from MoCap to World
        R_WM = Rotation.from_quat(mocap_q).as_matrix()
        # R_MB = np.array([[0.0, 1.0, 0.0],
        #                 [0.0, 0.0, 1.0],
        #                 [1.0, 0.0, 0.0]])
        # R_WB = R_WM @ R_MB

        # p_BM_B = np.array([0.037,0,0.1075]) #body to tracker in body
        # p_WB_W = p_WMo_W + R_WB @ ( -p_BM_B )
        
        self.R_WB = R_WM
        # self.CoM_pos = np.array([msg.position.x, msg.position.y, msg.position.z]) + p_offset
        self.CoM_pos = p_WMo_W

        # Publish robot position marker and update path if visualization is enabled
        if self.enable_visualization:
            self.publish_robot_position_marker(msg)
            self.update_robot_path(msg)

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
        
        # Check for trotting mode penetration detection and publish raw measurements
        try:
            ghost_behav_mode = self.spirit_state.behavior[1]
            
            if ghost_behav_mode > 1 and ghost_behav_mode < 1e6:  # Trotting mode
                # Check if FL penetration detected
                print("trotting")
                if self.penetration_detected_fl:
                    msg.front_left_leg.curr_pene = True
                    msg.front_left_leg.pene_time = self.spirit_state.mainboard_t
                    msg.front_left_leg.pene_depth = -self.forwardKinematicsSolver(self.jointVec[:,self.idx_fl], self.idx_fl)[2]
                    msg.front_left_leg.pene_force = self.spirit_state.joint_residuals[0]
                    msg.front_right_leg.curr_pene = False
                    msg.front_right_leg.pene_depth = float('nan')
                    msg.front_right_leg.pene_force = float('nan')
                    self.realtime_pene_publisher.publish(msg)
                    self.penetration_detected_fl = False  # Reset after publishing
                    return
                
                # Check if FR penetration detected
                if self.penetration_detected_fr:
                    msg.front_right_leg.curr_pene = True
                    msg.front_right_leg.pene_time = self.spirit_state.mainboard_t
                    msg.front_right_leg.pene_depth = -self.forwardKinematicsSolver(self.jointVec[:,self.idx_fr], self.idx_fr)[2]
                    msg.front_right_leg.pene_force = self.spirit_state.joint_residuals[4]
                    msg.front_left_leg.curr_pene = False
                    msg.front_left_leg.pene_depth = float('nan')
                    msg.front_left_leg.pene_force = float('nan')
                    self.realtime_pene_publisher.publish(msg)
                    self.penetration_detected_fr = False  # Reset after publishing
                    return
        except:
            pass
        
        # Original crawl mode logic - while when either of the leg is in penetration
        # remove pene_depth and pene_force data if not in penetration
        if not msg.front_left_leg.curr_pene:
            msg.front_left_leg.pene_depth = float('nan')
            msg.front_left_leg.pene_force = float('nan')
        if not msg.front_right_leg.curr_pene:
            msg.front_right_leg.pene_depth = float('nan')
            msg.front_right_leg.pene_force = float('nan')
        if msg.front_left_leg.curr_pene or msg.front_right_leg.curr_pene:
            self.realtime_pene_publisher.publish(msg)

    def spatial_measurement_publish(self):
        msg = SpatialMeasurement()
        if (self.pene_leg_idx == self.idx_fr) or (self.pene_leg_idx == self.idx_fl):

            # transform_to_map_T_MW = np.array(
            #     [[-1, 0, 0, 0],
            #      [ 0,-1, 0, 2.4],
            #      [ 0, 0, 1, 0],
            #      [ 0, 0, 0, 1]]
            # )
            p_WT_homo = np.zeros((4,1))
            p_WT_homo[0:3,0] = self.Toe_W[:,self.pene_leg_idx]
            # p_WT_homo[3,0]   = 1
            # p_MT_homo = transform_to_map_T_MW @ p_WT_homo

            msg.position.x = p_WT_homo[0,0]
            msg.position.y = p_WT_homo[1,0]
            msg.position.z = p_WT_homo[2,0]

        msg.uncertainty = self.r2_crawl  # Store R² as uncertainty measure
        msg.leg_idx = self.pene_leg_idx
        msg.value = self.stiffness
        msg.unit = "N/m"
        msg.source_name = "Crawling"
        msg.time = self.get_clock().now().to_msg()
        # print(self.pene_leg_idx)
        # print(msg)
        self.spatial_measurement_publisher.publish(msg)

    def publish_robot_position_marker(self, pose_msg):
        """Publish a marker showing robot position and head orientation (only called when visualization is enabled)"""
        marker = Marker()
        marker.header.frame_id = "map"  # or "world" depending on your frame setup
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set position from pose
        marker.pose.position.x = pose_msg.position.x
        marker.pose.position.y = pose_msg.position.y
        marker.pose.position.z = pose_msg.position.z

        # Set orientation to show head direction (apply 180-degree Z rotation for correct visualization)
        # Create a 180-degree rotation around Z-axis: q_flip = [0, 0, 1, 0]
        # Multiply: q_result = q_flip * q_original  
        orig_x, orig_y, orig_z, orig_w = pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w
        marker.pose.orientation.x = -orig_z
        marker.pose.orientation.y = orig_w  
        marker.pose.orientation.z = orig_x
        marker.pose.orientation.w = -orig_y

        # Set arrow size to represent robot scale (negative x to reverse arrow direction)
        marker.scale.x = -0.5  # Length of arrow (robot length) - negative to point forward
        marker.scale.y = 0.3  # Width of arrow shaft
        marker.scale.z = 0.3  # Height of arrow shaft

        # Set color - blue for robot
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Semi-transparent

        # Set lifetime (0 means forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.robot_marker_publisher.publish(marker)

    def update_robot_path(self, pose_msg):
        """Update and publish robot path trajectory (only called when visualization is enabled)"""
        # Create PoseStamped message for the path
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose_msg.position.x
        pose_stamped.pose.position.y = pose_msg.position.y
        pose_stamped.pose.position.z = pose_msg.position.z
        pose_stamped.pose.orientation.x = pose_msg.orientation.x
        pose_stamped.pose.orientation.y = pose_msg.orientation.y
        pose_stamped.pose.orientation.z = pose_msg.orientation.z
        pose_stamped.pose.orientation.w = pose_msg.orientation.w
        
        # Add to path
        self.robot_path.poses.append(pose_stamped)
        self.robot_path.header.stamp = self.get_clock().now().to_msg()
        
        # Limit path length to avoid memory issues
        if len(self.robot_path.poses) > 1000:
            self.robot_path.poses = self.robot_path.poses[-500:]
        
        # Publish path
        self.path_publisher.publish(self.robot_path)

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


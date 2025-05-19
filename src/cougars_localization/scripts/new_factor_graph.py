# your_gtsam_ros2_pkg/your_gtsam_ros2_pkg/factor_graph_node.py

import rclpy
from rclpy.node import Node
import numpy as np
import gtsam
from gtsam.symbol_shorthand import B, V, X
from functools import partial

# Import from sibling modules
from error_functions import error_depth, error_dvl, error_unary_heading, error_gps
from time_sync_fixed import TimeSync

# ROS Message types
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# Typing import removed as error functions are now external

class FactorGraphNode(Node):
    def __init__(self):
        super().__init__('factor_graph_node')

        # --- Parameters ---
        self.declare_parameter('imu_topic', 'modem_imu')
        self.declare_parameter('gps_topic', 'gps_odom')
        self.declare_parameter('dvl_topic', 'dvl/data')
        self.declare_parameter('depth_topic', 'depth_data')
        # self.declare_parameter('heading_topic', '/heading/pose')

        self.declare_parameter('gtsam_lag', 5.0)
        self.declare_parameter('factor_graph_period', 1.0)
        self.declare_parameter('imu_dt', 0.01) # Expected IMU data period if timestamps are unreliable

        # Sensor frequencies for TimeSync period calculation
        self.declare_parameter('gps_frequency_hz', 1.0)
        self.declare_parameter('dvl_frequency_hz', 6.0)
        self.declare_parameter('depth_frequency_hz', 20.0)
        self.declare_parameter('heading_frequency_hz', 10.0)

        # Noise parameters
        self.declare_parameter('prior_noise_sigma_pose_transl', 0.1)
        self.declare_parameter('prior_noise_sigma_pose_rot', 0.1)
        self.declare_parameter('vel_noise_sigma', 0.1)
        self.declare_parameter('pim_gyro_sigma', 1e-3)
        self.declare_parameter('pim_accel_sigma', 1e-3)
        self.declare_parameter('pim_integration_sigma', 1e-7)
        self.declare_parameter('bias_accel_between_sigma', 0.004)
        self.declare_parameter('bias_gyro_between_sigma', 0.004)
        self.declare_parameter('bias_prior_sigma_factor', 10.0)
        self.declare_parameter('gps_noise_sigma_xy', 0.5)
        self.declare_parameter('gps_noise_sigma_z', 1.0)
        self.declare_parameter('dvl_noise_sigmas', [0.1, 0.1, 0.1])
        self.declare_parameter('depth_noise_sigma', 0.1)
        self.declare_parameter('heading_noise_sigma_rad', np.deg2rad(3.0))
        
        self.declare_parameter('initial_bias_accel', [-0.3, 0.1, 0.2])
        self.declare_parameter('initial_bias_gyro', [0.1, 0.3, -0.1])

        # --- GTSAM Initialization ---
        self.lag = self.get_parameter('gtsam_lag').get_parameter_value().double_value
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.1)
        isam_params.relinearizeSkip = 1
        isam_params.findUnusedFactorSlots = True
        self.smoother = gtsam.IncrementalFixedLagSmoother(self.lag, isam_params)
        
        self.graph_new_factors = gtsam.NonlinearFactorGraph()
        self.initial_estimates_new_values = gtsam.Values()

        pim_params = gtsam.PreintegrationParams.MakeSharedU(9.81)
        gyro_sigma_val = self.get_parameter('pim_gyro_sigma').get_parameter_value().double_value
        accel_sigma_val = self.get_parameter('pim_accel_sigma').get_parameter_value().double_value
        integration_sigma_val = self.get_parameter('pim_integration_sigma').get_parameter_value().double_value
        I_3x3 = np.eye(3)
        pim_params.setGyroscopeCovariance(gyro_sigma_val**2 * I_3x3)
        pim_params.setAccelerometerCovariance(accel_sigma_val**2 * I_3x3)
        pim_params.setIntegrationCovariance(integration_sigma_val**2 * I_3x3)

        initial_bias_accel_val = self.get_parameter('initial_bias_accel').get_parameter_value().double_array_value
        initial_bias_gyro_val = self.get_parameter('initial_bias_gyro').get_parameter_value().double_array_value
        self.current_bias_estimate = gtsam.imuBias.ConstantBias(np.array(initial_bias_accel_val),
                                                               np.array(initial_bias_gyro_val))
        self.pim = gtsam.PreintegratedImuMeasurements(pim_params, self.current_bias_estimate)

        # Noise models
        prior_transl_sigma = self.get_parameter('prior_noise_sigma_pose_transl').get_parameter_value().double_value
        prior_rot_sigma = self.get_parameter('prior_noise_sigma_pose_rot').get_parameter_value().double_value
        self.PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.concatenate((np.full(3, prior_rot_sigma), np.full(3, prior_transl_sigma))))
        self.VEL_NOISE = gtsam.noiseModel.Isotropic.Sigma(
            3, self.get_parameter('vel_noise_sigma').get_parameter_value().double_value)
        bias_prior_factor = self.get_parameter('bias_prior_sigma_factor').get_parameter_value().double_value
        bias_accel_bet_sigma = self.get_parameter('bias_accel_between_sigma').get_parameter_value().double_value
        bias_gyro_bet_sigma = self.get_parameter('bias_gyro_between_sigma').get_parameter_value().double_value
        self.PRIOR_BIAS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.concatenate((np.full(3, bias_accel_bet_sigma * bias_prior_factor), 
                            np.full(3, bias_gyro_bet_sigma * bias_prior_factor))))
        self.BIAS_BETWEEN_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
            np.concatenate((np.full(3, bias_accel_bet_sigma), np.full(3, bias_gyro_bet_sigma))))
        gps_xy_sigma = self.get_parameter('gps_noise_sigma_xy').get_parameter_value().double_value
        gps_z_sigma = self.get_parameter('gps_noise_sigma_z').get_parameter_value().double_value
        self.GPS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([gps_xy_sigma, gps_xy_sigma, gps_z_sigma]))
        dvl_sigmas_val = self.get_parameter('dvl_noise_sigmas').get_parameter_value().double_array_value
        self.DVL_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array(dvl_sigmas_val))
        self.DEPTH_NOISE = gtsam.noiseModel.Isotropic.Sigma(
            1, self.get_parameter('depth_noise_sigma').get_parameter_value().double_value)
        self.HEADING_NOISE = gtsam.noiseModel.Isotropic.Sigma(
            3, self.get_parameter('heading_noise_sigma_rad').get_parameter_value().double_value)

        # State variables
        self.i = 0 # State index for X(i), V(i)
        self.bias_key_idx = 0
        self.current_time_from_imu = 0.0
        self.prev_nav_state_gtsam = None
        self.initialized = False

        # TimeSync instances for sensor data queuing and synchronization
        self.ts_gps = TimeSync()
        self.ts_dvl = TimeSync()
        self.ts_depth = TimeSync()
        self.ts_heading = TimeSync()

        self.posekey_to_time = {}
        self.velkey_to_time = {}
        self.biaskey_to_time = {}
        
        self.nodes_pending_gps_unary = []
        self.nodes_pending_depth_unary = []
        self.nodes_pending_heading_unary = []
        self.nodes_pending_dvl_unary = []

        self.last_gps_time = 0.0
        self.last_dvl_time = 0.0
        self.last_depth_time = 0.0
        self.last_heading_time = 0.0
        
        self.initial_gps_for_prior = None
        self.initial_heading_for_prior = None

        self.odom_publisher = self.create_publisher(Odometry, 'smoothed_output', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.imu_sub = self.create_subscription(ImuMsg, self.get_parameter('imu_topic').get_parameter_value().string_value, self.imu_callback, 100)
        self.gps_sub = self.create_subscription(NavSatFix, self.get_parameter('gps_topic').get_parameter_value().string_value, self.gps_callback, 10)
        self.dvl_sub = self.create_subscription(TwistWithCovarianceStamped, self.get_parameter('dvl_topic').get_parameter_value().string_value, self.dvl_callback, 10)
        self.depth_sub = self.create_subscription(PointStamped, self.get_parameter('depth_topic').get_parameter_value().string_value, self.depth_callback, 10)
        self.heading_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('heading_topic').get_parameter_value().string_value, self.heading_callback, 10)
        
        self.init_service = self.create_service(SetBool, 'init_factor_graph', self.init_factor_graph_callback)
        self.factor_graph_timer = self.create_timer(
            self.get_parameter('factor_graph_period').get_parameter_value().double_value,
            self.factor_graph_update_callback
        )
        self.get_logger().info("Factor Graph Node Initialized. Waiting for 'init_factor_graph' service call (true).")

    def init_factor_graph_callback(self, request: SetBool.Request, response: SetBool.Response):

        # Ensure TimeSync instances are also reset if de-initializing:
        if not request.data:
            self.initialized = False
            self.get_logger().info("Factor graph initialization has been unset.")
            # Reset states
            self.i = 0; self.bias_key_idx = 0; self.current_time_from_imu = 0.0
            self.prev_nav_state_gtsam = None
            self.ts_gps = TimeSync(); self.ts_dvl = TimeSync(); self.ts_depth = TimeSync(); self.ts_heading = TimeSync() # Reset TimeSync instances
            self.posekey_to_time.clear(); self.velkey_to_time.clear(); self.biaskey_to_time.clear()
            self.nodes_pending_gps_unary.clear(); self.nodes_pending_depth_unary.clear(); 
            self.nodes_pending_heading_unary.clear(); self.nodes_pending_dvl_unary.clear()
            # Reset smoother and graph containers
            isam_params = gtsam.ISAM2Params() # Re-fetch or store params if they can change
            isam_params.setRelinearizeThreshold(0.1); isam_params.relinearizeSkip = 1; isam_params.findUnusedFactorSlots = True
            self.smoother = gtsam.IncrementalFixedLagSmoother(self.lag, isam_params)
            self.graph_new_factors = gtsam.NonlinearFactorGraph()
            self.initial_estimates_new_values = gtsam.Values()
            # Reset bias and PIM
            initial_bias_accel_val = self.get_parameter('initial_bias_accel').get_parameter_value().double_array_value
            initial_bias_gyro_val = self.get_parameter('initial_bias_gyro').get_parameter_value().double_array_value
            self.current_bias_estimate = gtsam.imuBias.ConstantBias(np.array(initial_bias_accel_val), np.array(initial_bias_gyro_val))
            pim_params = gtsam.PreintegrationParams.MakeSharedU(9.81) # Re-fetch or store params
            gyro_sigma_val = self.get_parameter('pim_gyro_sigma').get_parameter_value().double_value
            accel_sigma_val = self.get_parameter('pim_accel_sigma').get_parameter_value().double_value
            integration_sigma_val = self.get_parameter('pim_integration_sigma').get_parameter_value().double_value
            I_3x3 = np.eye(3)
            pim_params.setGyroscopeCovariance(gyro_sigma_val**2 * I_3x3)
            pim_params.setAccelerometerCovariance(accel_sigma_val**2 * I_3x3)
            pim_params.setIntegrationCovariance(integration_sigma_val**2 * I_3x3)
            self.pim = gtsam.PreintegratedImuMeasurements(pim_params, self.current_bias_estimate)

            response.success = True; response.message = "Factor graph un-initialized and reset."; return response

        if self.initialized:
            self.get_logger().info("Factor graph already initialized.")
            response.success = True; response.message = "Already initialized."; return response

        initial_time = self.current_time_from_imu if self.current_time_from_imu > 0 \
                       else (self.get_clock().now().nanoseconds * 1e-9)
        
        initial_xyz = np.array([0.0, 0.0, 0.0])
        if self.initial_gps_for_prior:
            initial_xyz = self.initial_gps_for_prior.vector()
            self.get_logger().info(f"Using initial GPS for prior: {initial_xyz}")
        else:
            self.get_logger().warn("Initial GPS for prior not available. Using (0,0,Z_from_depth or 0).")
            # Check if ts_depth has any messages. Requires peeking or a get_last method.
            # For simplicity, this example won't peek into TimeSync's internal queue for initial Z.
            # A more robust way would be to process sensor queues once before this init if needed.

        initial_orientation = self.initial_heading_for_prior if self.initial_heading_for_prior else gtsam.Rot3()
        if self.initial_heading_for_prior: self.get_logger().info("Using initial heading for prior.")
        else: self.get_logger().warn("Initial heading for prior not available. Using identity rotation.")

        initial_pose_gtsam = gtsam.Pose3(initial_orientation, initial_xyz)
        initial_velocity_np = np.zeros(3)
        
        self.graph_new_factors.push_back(gtsam.PriorFactorPose3(X(0), initial_pose_gtsam, self.PRIOR_POSE_NOISE))
        self.graph_new_factors.push_back(gtsam.PriorFactorVector(V(0), initial_velocity_np, self.VEL_NOISE))
        self.graph_new_factors.push_back(gtsam.PriorFactorConstantBias(B(0), self.current_bias_estimate, self.PRIOR_BIAS_NOISE))

        self.initial_estimates_new_values.insert(X(0), initial_pose_gtsam)
        self.initial_estimates_new_values.insert(V(0), initial_velocity_np)
        self.initial_estimates_new_values.insert(B(0), self.current_bias_estimate)

        current_timestamps_for_update = gtsam.FixedLagSmootherKeyTimestampMap()
        current_timestamps_for_update.insert(X(0), initial_time)
        current_timestamps_for_update.insert(V(0), initial_time)
        current_timestamps_for_update.insert(B(0), initial_time)
        
        self.posekey_to_time[X(0)] = initial_time
        self.velkey_to_time[V(0)] = initial_time
        self.biaskey_to_time[B(0)] = initial_time

        self.prev_nav_state_gtsam = gtsam.NavState(initial_pose_gtsam, initial_velocity_np)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = "world"; t.child_frame_id = "odom_gtsam"
        t.transform.translation.x = initial_pose_gtsam.x(); t.transform.translation.y = initial_pose_gtsam.y(); t.transform.translation.z = initial_pose_gtsam.z()
        q = initial_pose_gtsam.rotation().toQuaternion()
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y(); t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w()
        self.tf_broadcaster.sendTransform(t)

        try:
            self.smoother.update(self.graph_new_factors, self.initial_estimates_new_values, current_timestamps_for_update)
            self.graph_new_factors = gtsam.NonlinearFactorGraph()
            self.initial_estimates_new_values.clear()
        except Exception as e:
            self.get_logger().error(f"Initial GTSAM update failed: {e}")
            response.success = False; response.message = f"Initial GTSAM update failed: {e}"; return response

        self.i = 1
        self.bias_key_idx = 0
        self.initialized = True
        self.get_logger().info(f"Factor Graph Initialized with priors at time {initial_time:.2f}s.")
        response.success = True; response.message = "Factor graph initialized successfully."; return response


    def imu_callback(self, msg: ImuMsg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if not self.initial_heading_for_prior: # Store first IMU orientation for prior if not set by dedicated heading sensor
             q_orient = msg.orientation
             if abs(q_orient.w) > 1e-6 or abs(q_orient.x) > 1e-6 or abs(q_orient.y) > 1e-6 or abs(q_orient.z) > 1e-6 : # Check if quaternion is not zero
                self.initial_heading_for_prior = gtsam.Rot3(q_orient.w, q_orient.x, q_orient.y, q_orient.z)

        if not self.initialized:
            self.current_time_from_imu = timestamp
            return

        dt_imu = self.get_parameter('imu_dt').get_parameter_value().double_value # Default dt
        if self.current_time_from_imu > 0 and timestamp > self.current_time_from_imu:
            dt_imu = timestamp - self.current_time_from_imu # Calculated dt
        
        if dt_imu <= 1e-9:
             self.get_logger().warn(f"IMU dt is {dt_imu:.4f}, skipping integration. Current time: {timestamp:.2f}, Prev time: {self.current_time_from_imu:.2f}")
             if timestamp > self.current_time_from_imu: self.current_time_from_imu = timestamp
             return

        self.current_time_from_imu = timestamp
        measured_omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        measured_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.pim.integrateMeasurement(measured_acc, measured_omega, dt_imu)

    #TODO: work on these callbacks to function properly with new msg types

    def gps_callback(self, msg: Odometry):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if timestamp <= self.last_gps_time: return

        local_x = msg.pose.pose.x  
        local_y = msg.pose.pose.y  
        local_z = msg.pose.pose.z  
        gps_meas_point3 = gtsam.Point3(local_x, local_y, local_z) # This is what error_gps expects

        if not self.initial_gps_for_prior:
            self.initial_gps_for_prior = gps_meas_point3
            self.get_logger().info(f"Stored initial GPS for prior (local XYZ): {local_x:.2f}, {local_y:.2f}, {local_z:.2f}")
        
        self.ts_gps.add_to_queue((gps_meas_point3, timestamp)) # Store (data, timestamp) for TimeSync
        self.last_gps_time = timestamp

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if timestamp <= self.last_dvl_time: return
        dvl_vel_vector = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        # dvl_vel_vector is what error_dvl expects
        self.ts_dvl.add_to_queue((dvl_vel_vector, timestamp))
        self.last_dvl_time = timestamp

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if timestamp <= self.last_depth_time: return
        depth_meas_for_error_func = np.array([msg.pose.pose.point.z]) # error_depth expects np.array([value])
        self.ts_depth.add_to_queue((depth_meas_for_error_func, timestamp))
        self.last_depth_time = timestamp

    def heading_callback(self, msg: PoseWithCovarianceStamped):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if timestamp <= self.last_heading_time: return
        q = msg.pose.pose.orientation
        orientation_gtsam_rot3 = gtsam.Rot3(q.w, q.x, q.y, q.z)
        
        if not self.initial_heading_for_prior:
            self.initial_heading_for_prior = orientation_gtsam_rot3
        
        # error_unary_heading expects measurement as a list: [gtsam.Rot3]
        heading_meas_for_error_func = [orientation_gtsam_rot3]
        self.ts_heading.add_to_queue((heading_meas_for_error_func, timestamp))
        self.last_heading_time = timestamp

    def factor_graph_update_callback(self):
        if not self.initialized or self.pim.deltaTij() == 0: return

        new_node_time = self.current_time_from_imu 
        active_bias_key = B(self.bias_key_idx)
        
        estimated_bias_for_prediction = self.current_bias_estimate
        if self.smoother.getISAM2Result().exists(active_bias_key):
             estimated_bias_for_prediction = self.smoother.getISAM2Result().atConstantBias(active_bias_key)
        
        if self.prev_nav_state_gtsam is None: self.get_logger().error("prev_nav_state_gtsam is None."); return

        predicted_nav_state = self.pim.predict(self.prev_nav_state_gtsam, estimated_bias_for_prediction)

        self.initial_estimates_new_values.insert(X(self.i), predicted_nav_state.pose())
        self.initial_estimates_new_values.insert(V(self.i), predicted_nav_state.velocity())
        
        current_timestamps_for_update = gtsam.FixedLagSmootherKeyTimestampMap()
        current_timestamps_for_update.insert(X(self.i), new_node_time)
        current_timestamps_for_update.insert(V(self.i), new_node_time)
        
        self.posekey_to_time[X(self.i)] = new_node_time
        self.velkey_to_time[V(self.i)] = new_node_time
        
        imu_factor = gtsam.ImuFactor(X(self.i - 1), V(self.i - 1), X(self.i), V(self.i), active_bias_key, self.pim)
        self.graph_new_factors.push_back(imu_factor)
        self.pim.resetIntegration()

        if self.i > 0 and self.i % 2 == 0 :
            new_bias_idx_val = self.bias_key_idx + 1
            self.initial_estimates_new_values.insert(B(new_bias_idx_val), estimated_bias_for_prediction)
            bias_between_factor = gtsam.BetweenFactorConstantBias(active_bias_key, B(new_bias_idx_val), gtsam.imuBias.ConstantBias(), self.BIAS_BETWEEN_NOISE)
            self.graph_new_factors.push_back(bias_between_factor)
            current_timestamps_for_update.insert(B(new_bias_idx_val), new_node_time)
            self.biaskey_to_time[B(new_bias_idx_val)] = new_node_time
            self.bias_key_idx = new_bias_idx_val
        
        # Add current node to pending lists
        if X(self.i) not in self.nodes_pending_gps_unary: self.nodes_pending_gps_unary.append(X(self.i))
        if X(self.i) not in self.nodes_pending_depth_unary: self.nodes_pending_depth_unary.append(X(self.i))
        if X(self.i) not in self.nodes_pending_heading_unary: self.nodes_pending_heading_unary.append(X(self.i))
        if V(self.i) not in self.nodes_pending_dvl_unary: self.nodes_pending_dvl_unary.append(V(self.i))

        min_time_for_pending_nodes = new_node_time - self.lag - 1.0 # Cutoff for processing old pending nodes
        
        # GPS Unary Factors
        dt_gps = 1.0 / self.get_parameter('gps_frequency_hz').get_parameter_value().double_value
        self.nodes_pending_gps_unary = [k for k in self.nodes_pending_gps_unary if self.posekey_to_time.get(k, -1) >= min_time_for_pending_nodes]
        gps_unary_matches, self.nodes_pending_gps_unary = \
            self.ts_gps.time_match_unaries_to_nodes(self.nodes_pending_gps_unary, self.posekey_to_time, dt_gps)
        for meas_data, key, _ in gps_unary_matches:
            node_time = self.posekey_to_time.get(key)
            if node_time is not None and abs(node_time - new_node_time) <= self.smoother.smootherLag():
                self.graph_new_factors.add(gtsam.CustomFactor(self.GPS_NOISE, [key], partial(error_gps, meas_data)))

        # Depth Unary Factors
        dt_depth = 1.0 / self.get_parameter('depth_frequency_hz').get_parameter_value().double_value
        self.nodes_pending_depth_unary = [k for k in self.nodes_pending_depth_unary if self.posekey_to_time.get(k, -1) >= min_time_for_pending_nodes]
        depth_unary_matches, self.nodes_pending_depth_unary = \
            self.ts_depth.time_match_unaries_to_nodes(self.nodes_pending_depth_unary, self.posekey_to_time, dt_depth)
        for meas_data, key, _ in depth_unary_matches: # meas_data is np.array([depth_value])
            node_time = self.posekey_to_time.get(key)
            if node_time is not None and abs(node_time - new_node_time) <= self.smoother.smootherLag():
                self.graph_new_factors.add(gtsam.CustomFactor(self.DEPTH_NOISE, [key], partial(error_depth, meas_data)))
        
        # Heading Unary Factors
        dt_heading = 1.0 / self.get_parameter('heading_frequency_hz').get_parameter_value().double_value
        self.nodes_pending_heading_unary = [k for k in self.nodes_pending_heading_unary if self.posekey_to_time.get(k, -1) >= min_time_for_pending_nodes]
        heading_unary_matches, self.nodes_pending_heading_unary = \
            self.ts_heading.time_match_unaries_to_nodes(self.nodes_pending_heading_unary, self.posekey_to_time, dt_heading)
        for meas_data, key, _ in heading_unary_matches: # meas_data is [gtsam.Rot3]
            node_time = self.posekey_to_time.get(key)
            if node_time is not None and abs(node_time - new_node_time) <= self.smoother.smootherLag():
                self.graph_new_factors.add(gtsam.CustomFactor(self.HEADING_NOISE, [key], partial(error_unary_heading, meas_data)))

        # DVL Unary Factors
        dt_dvl = 1.0 / self.get_parameter('dvl_frequency_hz').get_parameter_value().double_value
        self.nodes_pending_dvl_unary = [k for k in self.nodes_pending_dvl_unary if self.velkey_to_time.get(k, -1) >= min_time_for_pending_nodes]
        dvl_unary_matches, self.nodes_pending_dvl_unary = \
            self.ts_dvl.time_match_unaries_to_nodes(self.nodes_pending_dvl_unary, self.velkey_to_time, dt_dvl)
        for meas_data, key, _ in dvl_unary_matches: # meas_data is np.array([vx,vy,vz])
            node_time = self.velkey_to_time.get(key)
            if node_time is not None and abs(node_time - new_node_time) <= self.smoother.smootherLag():
                 self.graph_new_factors.add(gtsam.CustomFactor(self.DVL_NOISE, [key], partial(error_dvl, meas_data)))

        try:
            if not self.graph_new_factors.empty() or not self.initial_estimates_new_values.empty():
                self.smoother.update(self.graph_new_factors, self.initial_estimates_new_values, current_timestamps_for_update)
                result = self.smoother.calculateEstimate()
                self.prev_nav_state_gtsam = gtsam.NavState(result.atPose3(X(self.i)), result.atVector(V(self.i)))
                if result.exists(B(self.bias_key_idx)):
                    self.current_bias_estimate = result.atConstantBias(B(self.bias_key_idx))
                
                current_pose_gtsam = result.atPose3(X(self.i))
                current_vel_gtsam_world = result.atVector(V(self.i))
                odom_msg = Odometry()
                odom_msg.header.stamp = rclpy.time.Time(seconds=self.posekey_to_time[X(self.i)]).to_msg()
                odom_msg.header.frame_id = "odom_gtsam"; odom_msg.child_frame_id = "base_link_gtsam"
                odom_msg.pose.pose.position.x = current_pose_gtsam.x(); odom_msg.pose.pose.position.y = current_pose_gtsam.y(); odom_msg.pose.pose.position.z = current_pose_gtsam.z()
                q_odom = current_pose_gtsam.rotation().toQuaternion()
                odom_msg.pose.pose.orientation.x = q_odom.x(); odom_msg.pose.pose.orientation.y = q_odom.y(); odom_msg.pose.pose.orientation.z = q_odom.z(); odom_msg.pose.pose.orientation.w = q_odom.w()
                v_body = current_pose_gtsam.rotation().unrotate(current_vel_gtsam_world)
                odom_msg.twist.twist.linear.x = v_body[0]; odom_msg.twist.twist.linear.y = v_body[1]; odom_msg.twist.twist.linear.z = v_body[2]
                self.odom_publisher.publish(odom_msg)
            else: # Still update prev_nav_state if PIM was integrated but no optimization
                self.prev_nav_state_gtsam = predicted_nav_state 
        except Exception as e:
            self.get_logger().error(f"GTSAM update or Odom publishing failed: {e}", exc_info=True)

        self.graph_new_factors = gtsam.NonlinearFactorGraph()
        self.initial_estimates_new_values.clear() 
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
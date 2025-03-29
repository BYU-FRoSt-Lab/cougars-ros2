#!/usr/bin/env python3

import gtsam.imuBias
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np
from functools import partial
import gtsam
from typing import List, Optional
from gtsam.symbol_shorthand import L
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from gtsam.symbol_shorthand import B, V, X

TODO = False

def vector3(x, y, z):
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=float)

g = 9.81
n_gravity = vector3(0, 0, -g)

def preintegration_parameters():
    # IMU preintegration parameters
    # Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
    PARAMS = gtsam.PreintegrationParams.MakeSharedU(g)
    I = np.eye(3)
    PARAMS.setAccelerometerCovariance(I * 0.1)
    PARAMS.setGyroscopeCovariance(I * 0.1)
    PARAMS.setIntegrationCovariance(I * 0.1)
    PARAMS.setUse2ndOrderCoriolis(False)
    PARAMS.setOmegaCoriolis(vector3(0, 0, 0))

    BIAS_COVARIANCE = gtsam.noiseModel.Isotropic.Variance(6, 0.1)
    DELTA = gtsam.Pose3(gtsam.Rot3.Rodrigues(0, 0, 0),
                  gtsam.Point3(0.05, -0.10, 0.20))

    return PARAMS, BIAS_COVARIANCE, DELTA

class TimeSync:
    def __init__(self) -> None:

        self.msg_queue = []         # append message of specific sensor to queue
        self.last_key = None

    def add_to_queue(self, msg):
        self.msg_queue.append(msg)

    def get_factor_info(self, init_new_id, key_to_time):
        ''''
        new_id = int, The agent posekey id that you will start searching at
        poseKey_to_time = dict being tracked by factor graph. keys: poseKey values: corresponding timestamp
        '''

        # A flag to determine if all remaining measurements are in the future relative to the current pose key's time
        in_future = False

        new_id = init_new_id
        curr_time = key_to_time[int(new_id)] 
        key_to_add = None
        msg_to_add = None

        # If measurement in queue and the oldest measurment is later than current posekey
        # Process the oldest measurements in self.msg_queue to decide if they are relevant for the current new_id
        while(len(self.msg_queue) > 1 and in_future is False):       

            oldest_measurement_time = (self.msg_queue[0].header.stamp.sec * 1_000_000_000 + self.msg_queue[0].header.stamp.nanosec) 
            next_measurement_time = (self.msg_queue[1].header.stamp.sec * 1_000_000_000 + self.msg_queue[1].header.stamp.nanosec) 
            # print("Curr", curr_time, "Oldest", oldest_measurement_time, "Next", next_measurement_time)

            if(oldest_measurement_time < curr_time):
                # print('oldest measurement time < curr time')
                
                newer_key_time = key_to_time[int(new_id)] 
                # print("newer key time: %d\n"%newer_key_time)
                print("Timesync new id:", int(new_id - 1))
                older_key_time = key_to_time[int(new_id - 1)]
                # print("older key time: %d\n"% older_key_time)
                time_to_current = abs(newer_key_time - oldest_measurement_time) 
                # print("time to current: %d\n"%time_to_current)
                time_to_previous = abs(older_key_time - oldest_measurement_time) 
                # print("time to previous: %d\n"%time_to_previous)

                if(time_to_current > time_to_previous):
                    # Meas in queue better suited to previous key, keep working on prev key
                    print('time to current is longer than time to prev')
                    print('new', new_id, ' prev', self.last_key, ' init', init_new_id)
                    new_id -= 1
                    if(new_id == self.last_key):
                        print('pop')
                        self.msg_queue.pop(0)
                        new_id = init_new_id
                else:
                    # print('time to current is shortest')

                    time_old_to_pose = abs(oldest_measurement_time - newer_key_time)
                    time_next_to_pose = abs(next_measurement_time - newer_key_time)

                    if(next_measurement_time < newer_key_time):
                    # Take care of where next measurement is not past next node
                        self.msg_queue.pop(0)
                    
                    elif(time_old_to_pose > time_next_to_pose):
                    # Take care of case where next measurement is better
                        self.msg_queue.pop(0)
                        new_id = init_new_id

                    else:
                        # Actually add the factor
                        key_to_add = new_id
                        msg_to_add = self.msg_queue.pop(0)                    
                        self.last_key = new_id
            else:
                in_future = True

        return key_to_add, msg_to_add
    
class FactorGraphNode(Node):

    def __init__(self):
        super().__init__('new_factor_graph_node')

        self.params, self.bias_covariance, self.delta = preintegration_parameters()
        self.p_imu = gtsam.PreintegratedImuMeasurements(self.params)

        self.q_gps = TimeSync()
        self.q_depth = TimeSync()
        self.q_dvl = TimeSync()
        self.q_imu = TimeSync()

        self.poseKey_to_time = {}
        self.biasKey_to_time = {}
        self.velKey_to_time = {}

        self.key = int(0)
       
        # factor graph period
        self.factor_graph_period = 2


        # flags to start the whole system
        self.depth_received = False
        self.gps_received = False
        self.imu_received = False
        self.dvl_received = False
    
        # gtsam stuff

        # noise models
        self.std_pose = np.array([0.001, 0.001, 0.001, np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        self.DVL_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_pose)
        self.std_gps = 6.0
        # TODO: make the z on gps really  high covariance, ignore the 
        self.GPS_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, self.std_gps) 
        self.std_orientation = np.deg2rad(3)
        self.UNARY_HEADING_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, self.std_orientation)
        self.std_depth = 0.1
        self.DEPTH_NOISE = gtsam.noiseModel.Isotropic.Sigma(1,self.std_depth )

        # incremental smoothing and mapping (isam) optimizer
        self.isam = gtsam.ISAM2()
        # factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        # values in factor graph
        self.initialEstimate = gtsam.Values()

        # Initialize class variables
        self.orientation_matrix = np.eye(3)
        self.orientation_covariance = np.zeros((3, 3))
        self.position = np.zeros(3)
        self.position_covariance = np.zeros((3, 3))
        self.dvl_velocity = np.zeros(3)
        self.dvl_velocity_covariance = np.zeros((3, 3))
        self.init_state = {}

        # flag to indicate that we have added a prior factor to pose 1
        # we now will begin the timer to add between factors and unary factors
        self.deployed = False

        # Subscribers

        # sensor subscriptions
        self.create_subscription(Imu, 'modem_imu', self.imu_callback, 10) # for unary factor
        self.create_subscription(PoseWithCovarianceStamped, 'depth_data', self.depth_callback, 10) # for unary factor
        self.create_subscription(Odometry, 'gps_odom', self.gps_callback, 10) # for unary factor
        self.create_subscription(TwistWithCovarianceStamped, 'dvl_velocity', self.dvl_callback, 10) # for between factor (dead reckon. pose to pose)
        self.create_subscription(Empty, 'init', self.add_prior_callback, 10)

        # Publisher

        # publishes the LATEST output of the smoothing and mapping (most importantly gps x,y), although remember the whole path is being smoothed
        self.state_est_pub = self.create_publisher(Odometry, 'smoothed_output', 10)
        self.state_est_msg = Odometry()
        
        # Timer for adding state estimate/factors
        self.timer = self.create_timer(self.factor_graph_period, self.factor_graph_timer)
        
        # Initialize the transform broadcaster

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        

    # error functions for unary factors
    def get_unary_heading(self, pose, measurement):

        rot_pred = pose.rotation()

        # Get full relative pose meas
        rot_meas = measurement[0]

        # print("rot pred", rot_pred)
        # print("rot meas", rot_meas)

        error = gtsam.Rot3.Logmap(rot_pred.between(rot_meas))
        # print("between anchor error", error)

        return error

    def error_unary_heading(self, measurement: np.ndarray, this: gtsam.CustomFactor,
                values: gtsam.Values,
                jacobians: Optional[List[np.ndarray]]) -> float:
        
        key_pose = this.keys()[0]
        pose = values.atPose3(key_pose)

        if jacobians is not None:
            eps = 1e-6

            # Six rows bc error state, six columns bc the state of this node is Pose3
            # Perturb relative to robot A pose key
            H0 = np.zeros((3, 6))
            for i in range(6):
                delta_step = np.zeros(6)
                delta_step[i] = eps
                delta_step_forward = gtsam.Pose3.Expmap(delta_step)
                delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

                poseA_f = pose.compose(delta_step_forward)
                poseA_b = pose.compose(delta_step_backward)

                error_forward = self.get_unary_heading(poseA_f, measurement)
                error_backward = self.get_unary_heading(poseA_b, measurement)
                hdot =  (error_forward - error_backward) / (2*eps)
                H0[:,i] = hdot

            jacobians[0] = H0

        error = self.get_unary_heading(pose, measurement)

        return error

    def error_gps(self, measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
        """GPS Factor error function
        :param measurement: GPS measurement, to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """

        # Get agent pose
        estimate = values.atPose3(this.keys()[0])

        # Three rows bc [x, y, z] from gps state, six columns bc the state of this node is Pose3
        H = np.zeros((3, 6))
        eps = 1e-6

        # Get Jacobians. Perturb in each direction of the delta vector one at a time.
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            q_exp_forward = estimate.compose(delta_step_forward)
            q_exp_backward = estimate.compose(delta_step_backward)
            h_forward = q_exp_forward.translation()
            h_backward = q_exp_backward.translation()
            error_forward = h_forward - measurement
            error_backward = h_backward - measurement
            hdot =  (error_forward - error_backward) / (2*eps)
            H[:,i] = hdot

        
        error = estimate.translation() - measurement

        if jacobians is not None:
            # Error wrt agent A
            jacobians[0] = H
    

        return error
    
    def error_depth(self, measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
        """Depth Factor error function
        :param measurement: Depth measurement, to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        pos = values.atPose3(this.keys()[0])

        # Depth error (z position). TODO 1D or 2D array?
        error = pos.translation()[2] - measurement[0]
        # print("depth pred", pos.translation()[2])
        # print("depth meas", pos.translation()[2])

        # print("depth error", error)

        # Jacobians
        if jacobians is not None:
            eps = 1e-6

            # Wrt pose key
            H0 = np.zeros((1, 6))

            # Get Jacobians. Perturb in each direction of the delta vector one at a time.
            for i in range(6):
                delta_step = np.zeros(6)
                delta_step[i] = eps
                delta_step_forward = gtsam.Pose3.Expmap(delta_step)
                delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

                # Perturb pose
                q_exp_forward = pos.compose(delta_step_forward)
                q_exp_backward = pos.compose(delta_step_backward)

                # Extract depth
                h_forward = q_exp_forward.translation()[2]
                h_backward = q_exp_backward.translation()[2]

                # Compute error
                error_forward = h_forward - measurement[0]
                error_backward = h_backward - measurement[0]
                hdot =  (error_forward - error_backward) / (2*eps)
                H0[:,i] = hdot

            jacobians[0] = H0

        return error
    
    def error_dvl(self, measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
        
        """DVL factor Factor error function
        :param measurement: Odometry measurement, to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        key = this.keys()[0]
    
        vel = values.atVector(key)
        error = vel - measurement
        if jacobians is not None:
            jacobians[0] = -np.eye(1) 
            jacobians[1] = np.eye(1) 

        return error

    def HfromRT(self, R, t):
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = t
        return H

    def update(self):
        self.isam.update(self.graph, self.initialEstimate)
        self.result = self.isam.calculateEstimate()
        self.est_xyz = self.result.atPose3(X(self.key)).translation()
        self.est_orientation = self.result.atPose3(X(self.key)).orientation()
        self.isam.update()
        self.isam.update()
        self.isam.update()
        self.initialEstimate.clear()
        self.graph.resize(0)

    
    ##################################################################
    ######################## SENSOR CALLBACKS ########################
    ##################################################################

    # IMU -- odometry measurements for pre-integrated imu measurements
    def parse_imu_data(self, msg: Imu):
        self.latest_imu_orientation_covariance = np.array(msg.orientation_covariance).reshape(3, 3)
        # Convert quaternion to rotation matrix
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(quat)
        self.angular_velocity = vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        self.linear_acceleration = vector3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.latest_imu_orientation_matrix = r.as_matrix()
        self.latest_imu_factor_time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9

    def imu_callback(self, msg: Imu):
        self.imu_received = True
        self.latest_imu_msg = msg
        self.parse_imu_data(self.latest_imu_msg)

    # depth -- to add depth unary factors to pose estimates
    def parse_depth_data(self, msg: PoseWithCovarianceStamped):
        self.latest_depth = msg.pose.pose.position.z
        self.latest_time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        self.depth_received = True
        if self.deployed:
            self.q_depth.add_to_queue(msg)
        else:
            self.latest_depth_msg = msg
            self.parse_depth_data(msg)

    # gps -- to add gps unary factors to pose estimates
    def parse_gps(self, msg: Odometry):
        self.latest_gps_x = msg.pose.pose.position.x
        self.latest_gps_y = msg.pose.pose.position.y
        self.latest_gps_z = msg.pose.pose.position.z
        # self.position_covariance = np.array(msg.pose.covariance).reshape(3, 3)
        #Plot
        self.gps_time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9

    def gps_callback(self, msg: Odometry):
        self.gps_received = True
        if self.deployed:
            self.q_gps.add_to_queue(msg)
        else:
            self.latest_gps_msg = msg
            self.parse_gps(msg)
   
    # dvl -- to add dvl unary factors to velocity estimates
    def parse_dvl(self, msg: TwistWithCovarianceStamped):
        self.latest_dvl_vel_x = msg.twist.twist.linear.x
        self.latest_dvl_vel_y = msg.twist.twist.linear.y
        self.latest_dvl_vel_z = msg.twist.twist.linear.z
        self.latest_dvl_time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        self.dvl_received = True
        if self.deployed:
            self.q_dvl.add_to_queue(msg)
        else:
            self.latest_dvl_msg = msg
            self.parse_dvl(msg)

    

    ##################################################################
    ######################## SIGNAL TO BEGIN #########################
    ##################################################################

    def add_prior_callback(self, msg: Empty):

        if self.gps_received and self.dvl_received and self.depth_received and self.imu_received:
            # set deployed flag to true
            self.deployed = True

            # Initial values to be used in initial estimatepfjpj
            orientation_matrix = self.latest_imu_orientation_matrix # from the IMU at the beginning
            translation = vector3(self.latest_gps_x, self.latest_gps_y, self.latest_depth)
            velocities = vector3(self.latest_dvl_vel_x, self.latest_dvl_vel_y, self.latest_dvl_vel_z)

            # add X prior, V prior, and B prior, and attach to initial estimate nodes

            # X: Pose3 prior -- Translation from GPS, orientation from IMU
            poseKey = X(self.key)
            pose_0 = self.HfromRT(orientation_matrix, translation)
            noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]))
            self.graph.push_back(gtsam.PriorFactorPose3(poseKey, pose_0, noise))
            self.initialEstimate.insert(poseKey, pose_0)
            self.poseKey_to_time[poseKey] = self.latest_imu_factor_time

            # B: Bias prior
            biasKey = B(self.key)
            biasnoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
            biasprior = gtsam.PriorFactorConstantBias(biasKey, gtsam.imuBias.ConstantBias(),biasnoise)
            self.graph.push_back(biasprior)
            self.initialEstimate.insert(biasKey, gtsam.imuBias.ConstantBias())
            self.biasKey_to_time[biasKey] = self.latest_imu_factor_time

            # V: Velocity prior -- from DVL
            velKey = V(self.key)
            self.velnoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
            velprior = gtsam.PriorFactorVector(velKey, velocities, self.velnoise)
            self.graph.push_back(velprior)
            self.initialEstimate.insert(velKey, velocities)
            self.velKey_to_time[velKey] = self.latest_imu_factor_time

            # first factor graph update
            self.update()
            self.publish_state_est()
        else:
            # print 
            self.get_logger().warning("Have not received all necessary sensor inputs to begin")
            self.get_logger().warning(f"IMU: {self.imu_received}\nGPS: {self.gps_received}\nDVL: {self.dvl_received}\nDepth: {self.depth_received}")

        


    ##################################################################
    ####################### factor graph stuff #######################
    ##################################################################
    
    def factor_graph_timer(self):
        # Your timer callback function
        # self.get_logger().info('factor_graph_timer function is called')
        if self.deployed:
            
            # increment key  
            self.key += 1
        

            
            time_stamp_to_query = self.get_clock().now().to_msg()
            # Tf2 query (need to query tranformations at the the current time, and need to transform the below imu_vel, 
            # imu_pose, vel3D, gps_xyz to the world frame in terms of the shared origin between all the agents)

            #     ...
            #     
            #     
            #     
            #     ...
            #     
            #     
            #     
            #     ...
            #     
            #     
            #     
            #     ...
            #     
            #     
            #     
            #     ...


            #####################################################################

            # Time sorted IMU linear acceleration and angualr velocity
            self.measuredLinAcc = self.linear_acceleration
            self.measuredAngOmega = self.angular_velocity
            self.p_imu.integrateMeasurement(self.measuredLinAcc, self.measuredAngOmega, self.factor_graph_period)
            self.predicted_state = self.runner.predict(self.p_imu, self.actualBias)
            
            # Time sorted DVL velocity (with x, y, and z)
            self.imu_vel = self.predicted_state.velocity()

            #  Integrate from IMU?
            self.imu_pose = self.predicted_state.pose()
            
            # add new bias between factor and bias node
            bias_factor = gtsam.BetweenFactorConstantBias(B(self.key - 1) , B(self.key), gtsam.imuBias.ConstantBias(), self.bias_covariance)
            self.graph.add(bias_factor)
            self.initialEstimate.insert(B(self.key), gtsam.imuBias.ConstantBias())

            # add imu factors
            imu_factor = gtsam.ImuFactor(X(self.key - 1), V(self.key - 1), X(self.key), V(self.key), B(self.key), self.p_imu)
            self.graph.add(imu_factor)
           
            # add X node
            self.initialEstimate.insert(X(self.key), self.imu_pose)
            self.poseKey_to_time[X(self.key)] = self.latest_imu_factor_time

            # add V node
            self.initialEstimate.insert(V(self.key), self.imu_vel)
            self.poseKey_to_time[V(self.key)] = self.latest_imu_factor_time


            # wait till at least 2 new X/V nodes are added to the graph

            if self.key > 2:

                # Add DVL unary factor to Velocity V Node, if there exists a time appropriate measurement

                velKey_to_add_to, msg_to_add = self.q_gps.get_factor_info(V(self.key), self.velKey_to_time)
                if velKey_to_add_to is not None and msg_to_add is not None:
                    self.parse_dvl(msg_to_add)
                    vel_3D = vector3(self.latest_dvl_vel_x, self.latest_dvl_vel_y, self.latest_dvl_vel_z)
                    self.graph.add(gtsam.CustomFactor(self.velnoise, [V(velKey_to_add_to)], partial(self.error_dvl, [vel_3D])))

                # Add GPS unary factor to Pose3D X Node, if there exists a time appropriate measurement
                poseKey_to_add_to, msg_to_add = self.q_gps.get_factor_info(X(self.key), self.poseKey_to_time)
                if poseKey_to_add_to is not None and msg_to_add is not None:
                    self.parse_gps(msg_to_add)
                    gps_xyz = vector3(self.latest_gps_x, self.latest_gps_y, self.latest_gps_z)
                    gps_meas = gtsam.Point3(gps_xyz[0], gps_xyz[1], gps_xyz[2])
                    self.graph.add(gtsam.CustomFactor(self.GPS_NOISE, [X(poseKey_to_add_to)], partial(self.error_gps, gps_meas)))

                # add Depth unary factor to Pose3D X Node, if there exists a time appropriate measurement
                poseKey_to_add_to, msg_to_add = self.q_depth.get_factor_info(int(X(self.key)), self.poseKey_to_time)
                if poseKey_to_add_to is not None and msg_to_add is not None:
                    self.parse_depth_data(msg_to_add)
                    if(self.latest_depth > -0.25):
                        self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [X(poseKey_to_add_to)], partial(self.error_depth, [np.array([self.latest_depth])])))

                # TODO: ??? Add Heading unary factor to Pose3D X Node, if there exists a time appropriate measurement
                #     Since we are already adding imu information to the graph, why also add heading?
                #     ...
                #     ...
                #     ...
                #     ...

    

            self.update()
            self.publish_state_est()


    # PUBLISH THE DATA

    def publish_state_est(self):
        
        # Set the orientation in the message
        r = R.from_matrix(self.est_orientation)
        quat = r.as_quat()
        self.state_est_msg.pose.pose.orientation.x = quat[0]
        self.state_est_msg.pose.pose.orientation.y = quat[1]
        self.state_est_msg.pose.pose.orientation.z = quat[2]
        self.state_est_msg.pose.pose.orientation.w = quat[3]

        # Set the position in the message
        
        # this is the only gtsam output right now, orientation and depth are raw from the sensors
        self.state_est_msg.pose.pose.position.x = self.est_xyz[0]
        self.state_est_msg.pose.pose.position.y = self.est_xyz[1]
        self.state_est_msg.pose.pose.position.z = self.est_xyz[2]

        # Publish the vehicle status
        self.state_est_pub.publish(self.state_est_msg)



def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
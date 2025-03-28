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
        self.last_pose_key = None

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
        curr_time = key_to_time[new_id] 
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
                    print('new', new_id, ' prev', self.last_pose_key, ' init', init_new_id)
                    new_id -= 1
                    if(new_id == self.last_pose_key):
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
                        self.last_pose_key = new_id
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
        self.xyz = self.result.atPose3(X(self.key)).translation()
        self.isam.update()
        self.isam.update()
        self.isam.update()
        self.initialEstimate.clear()
        self.graph.resize(0)

    
    ##################################################################
    ######################## SENSOR CALLBACKS ########################
    ##################################################################

    # IMU -- odometry measurements for pre-integrated imu measurements
    def imu_callback(self, msg: Imu):
        if self.deployed:
            self.q_imu.add_to_queue(msg)

        
    # d
    def depth_callback(self, msg: PoseWithCovarianceStamped):
        if self.deployed:
            self.q_depth.add_to_queue(msg)
    
    # gps (already in x,y from gps_odom.py)
    def gps_callback(self, msg: Odometry):
        if self.deployed:
            self.q_gps.add_to_queue(msg)
   

    # dvl for new odometry
    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        if self.deployed:
            self.q_dvl.add_to_queue(msg)
    

    ##################################################################
    ######################## SIGNAL TO BEGIN #########################
    ##################################################################

    def add_prior_callback(self, msg: Empty):
            
        self.deployed = True

        # TODO: gather initial values for these measurements:
        orientation_matrix = TODO # from the IMU at the beginning
        translation = TODO # From GPS at the beginning
        velocities = TODO # from the DVL

        # add X prior, V prior, and B prior, and attach to initial estimate nodes

        # X: Pose3 prior -- Translation from GPS, orientation from IMU
        poseKey = X(self.key)
        pose_0 = self.HfromRT(orientation_matrix, translation)
        noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]))
        self.graph.push_back(gtsam.PriorFactorPose3(poseKey, pose_0, noise))
        self.initialEstimate.insert(poseKey, pose_0)

        # B: Bias prior
        biasKey = B(self.key)
        biasnoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        biasprior = gtsam.PriorFactorConstantBias(biasKey, gtsam.imuBias.ConstantBias(),biasnoise)
        self.graph.push_back(biasprior)
        self.initialEstimate.insert(biasKey, gtsam.imuBias.ConstantBias())


        # V: Velocity prior -- from DVL
        velKey = V(self.key)
        velnoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        n_velocity = vector3(velocities.x, velocities.y, velocities.z)
        velprior = gtsam.PriorFactorVector(velKey, n_velocity, velnoise)
        self.graph.push_back(velprior)
        self.initialEstimate.insert(velKey, n_velocity)

                
        self.update()
        self.publish_state_est()
         
        


    ##################################################################
    ####################### factor graph stuff #######################
    ##################################################################
    
    def factor_graph_timer(self):
        # Your timer callback function
        # self.get_logger().info('factor_graph_timer function is called')
        if self.deployed:
            
            # increment key  
            self.key += 1
        
            #####################################################################
            # TODO: Time sort and tf2 below

            # Time sort

            #     ...
            #     ...
            #     ...
            #     ...
            #     ...
            #     ...

            time_stamp_to_query = self.get_clock().now().to_msg()

            # Tf2 query (should get IMU measuredLinAcc & measuredAngOmega, DVL xyz velocities, GPS x-y coordinates)

            #     ...
            #     ...
            #     ...
            #     ...
            #     ...
            #     ...

            #####################################################################

            # Time sorted IMU linear acceleration and angualr velocity
            self.measuredLinAcc = TODO
            self.measuredAngOmega = TODO
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

            # add V node
            self.initialEstimate.insert(V(self.key), self.imu_vel)


            if self.key > 2:
                pass

                # TODO: Add unary factors

                # Add DVL unary factor to Velocity V Node, if there exists a time appropriate measurement

                #     ...
                #     ...
                #     ...
                #     ...
                #     ...
                #     ...
                

                # Add GPS unary factor to Pose3D X Node, if there exists a time appropriate measurement

                #     ...
                #     ...
                #     ...
                #     ...
                #     ...
                #     ...


                # Add Heading unary factor to Pose3D X Node, if there exists a time appropriate measurement

                #     ...
                #     ...
                #     ...
                #     ...
                #     ...
                #     ...


                # add Depth unary factor to Pose3D X Node, if there exists a time appropriate measurement

                #     ...
                #     ...
                #     ...
                #     ...
                #     ...
                #     ...



            self.update()
            self.publish_state_est()




    # PUBLISH THE DATA

    def publish_state_est(self):
        
        # # Set the orientation in the message
        r = R.from_matrix(self.orientation_matrix)
        quat = r.as_quat()
        self.state_est_msg.pose.pose.orientation.x = quat[0]
        self.state_est_msg.pose.pose.orientation.y = quat[1]
        self.state_est_msg.pose.pose.orientation.z = quat[2]
        self.state_est_msg.pose.pose.orientation.w = quat[3]

        # # Set the position in the message
        
        # this is the only gtsam output right now, orientation and depth are raw from the sensors
        self.state_est_msg.pose.pose.position.x = self.xyz[0]
        self.state_est_msg.pose.pose.position.y = self.xyz[1]
        print("vehicle status xyz[0]: ", self.xyz[0])

        print("vehicle status x: ", self.state_est_msg.pose.pose.position.x)


        self.state_est_msg.pose.pose.position.z = self.position[2]


        # Publish the vehicle status
        print("PUBLISHING NEW FG")
        self.state_est_pub.publish(self.state_est_msg)


# x = result.atPose3(poseKey).translation()[0]


def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
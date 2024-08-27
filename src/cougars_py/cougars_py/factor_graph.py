import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np
from functools import partial
import gtsam
from typing import List, Optional
from gtsam.symbol_shorthand import L

class Agent():
    def __init__(self, H_init):
        self.pose_world_noisy = gtsam.Pose3(H_init)
        self.poseKey = int(1)
        self.prevPoseKey = self.poseKey

class FactorGraphNode(Node):

    def __init__(self):
        super().__init__('factor_graph_node')

        # number of seconds we take a dvl dead reck. pose
        self.dvl_time_interval = 0.25

        # measurement queues for sensors
        self.q_depth = []
        self.q_imu = []
        self.q_gps = []
        self.q_dvl = []

        # map of poseKeys to time stamps
        self.poseKey_to_time = {}
        
        # 'pointer' to mark the oldest pose that we shouldn't worry about adding a unary factor to
        # for example, if we add a gps unary factor to the second pose, than the gps_last_pose_key 
        # will point to the second pose. Or if gps goes out for a bit, this should point to the oldest
        # pose that we will not consider adding a unary factor to
        self.gps_last_pose_key = None       
        self.depth_last_pose_key = None
        self.imu_last_pose_key = None   

        # gtsam stuff

        # noise models
        self.std_pose = np.array([5, 5, 5, np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        self.DVL_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_pose)
        self.std_gps = 0.0001
        # TODO: make the z on gps really  high covariance, ignore the 
        self.GPS_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, self.std_gps) 
        self.std_orientation = np.deg2rad(3)
        self.UNARY_HEADING_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, self.std_orientation)

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
        self.dvl_position = np.zeros(3)
        self.dvl_position_covariance = np.zeros((3, 3))
        self.init_state = {}

        # flag to indicate that we have added a prior factor to pose 1
        # we now will begin the timer to add between factors and unary factors
        self.deployed = False

        # Subscribers

        # sensor subscriptions
        self.create_subscription(Imu, '/modem_imu', self.imu_callback, 10) # for unary factor
        self.create_subscription(PoseWithCovarianceStamped, '/depth_data', self.depth_callback, 10) # for unary factor
        self.create_subscription(Odometry, '/gps_odom', self.gps_callback, 10) # for unary factor
        self.create_subscription(Odometry, '/dvl_dead_reckoning', self.dvl_callback, 10) # for between factor (dead reckon. pose to pose)

        # signal to add prior (should be sent after DVL-lock and ref. gps/ heading is stored and DVL is restarted)
        self.create_subscription(Empty, '/init', self.init_callback, 10)

        # Publisher

        # publishes the LATEST output of the smoothing and mapping (most importantly gps x,y), although remember the whole path is being smoothed
        self.vehicle_status_pub = self.create_publisher(Odometry, '/smoothed_output', 10)
        self.odom_msg = Odometry()
        
        # Timer for adding state estimate/factors
        self.timer = self.create_timer(self.dvl_time_interval, self.factor_graph_timer)
    

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
        :param measurement: Landmark measurement, to be filled with `partial`
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


    # helper funciton to get H. matrix from rotation and translation
    def HfromRT(self, R, t):
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = t
        return H

    # update at every tick of the timer (after between factor and checking for unary factors)
    def update(self):
        self.isam.update(self.graph, self.initialEstimate)
        self.result = self.isam.calculateEstimate()
        self.xyz = self.result.atPose3(self.agent.poseKey).translation()  
        self.initialEstimate.clear()
        self.graph.resize(0)


    
    ##################################################################
    ######################## SENSOR CALLBACKS ########################
    ##################################################################


    # orientation from modem
    def imu_callback(self, msg: Imu):

        self.orientation_covariance = np.array(msg.orientation_covariance).reshape(3, 3)
        # Convert quaternion to rotation matrix
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(quat)
        self.orientation_matrix = r.as_matrix()

        if self.deployed:
            self.q_imu.append(msg)
        
    # depth sensor
    def depth_callback(self, msg: PoseWithCovarianceStamped):
        # Set the z position and covariance

        self.position[2] = msg.pose.pose.position.z
    
        if self.deployed:
            self.q_depth.append(msg)
    
    # gps (already in x,y from gps_odom.py)
    def gps_callback(self, msg: Odometry):
        # Get the x, y position and position covariance
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.gps_z = msg.pose.pose.position.z
        # self.position_covariance = np.array(msg.pose.covariance).reshape(3, 3)
        if self.deployed :
            self.q_gps.append(msg)
   

    # dvl for new odometry
    def dvl_callback(self, msg: Odometry):
        # Get the x, y, z position
        self.dvl_position[0] = msg.pose.pose.position.x
        self.dvl_position[1] = msg.pose.pose.position.y
        self.dvl_position[2] = msg.pose.pose.position.z
        self.dvl_std = msg.pose.covariance[0]
        # self.std_pose = np.array([self.dvl_std, self.dvl_std, self.dvl_std, np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        # self.DVL_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_pose)
        self.dvl_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.dvl_time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9

        # TODO: need covariance matrix, look into covariance, figure of merit

    

    ##################################################################
    ######################## SIGNAL TO BEGIN #########################
    ##################################################################

    def init_callback(self, msg: Empty):
        # Store current state as the initial state
        self.init_state = {
            'position': self.position.copy(),
            'orientation_matrix': self.orientation_matrix,
            'dvl_position': self.dvl_position.copy()
        }


        H = self.HfromRT(self.init_state['orientation_matrix'],self.init_state['position'])


        self.agent = Agent(H)

        self.dvl_pose_current = gtsam.Pose3(H)
        self.poseKey = int(1)
        self.prevPoseKey = self.poseKey

        

        priorFactor = gtsam.PriorFactorPose3(self.agent.poseKey, self.dvl_pose_current, self.DVL_NOISE)
        self.graph.push_back(priorFactor)
        self.initialEstimate.insert(self.agent.poseKey, self.dvl_pose_current)
        self.poseKey_to_time[self.agent.poseKey] = self.dvl_time
        self.gps_last_pose_key = self.agent.poseKey
        self.depth_last_pose_key = self.agent.poseKey
        self.imu_last_pose_key = self.agent.poseKey


        self.dvl_position_last = self.dvl_pose_current
    
        self.get_logger().info("Initial state has been set.")

        self.deployed = True

    
    ##################################################################
    ########### algorithms for time syncing measurements #############
    ##################################################################

    def unary_assignment(self, sensor):

        if sensor == 'gps':
            
            curr_time = self.dvl_time       #Timestamp of the current pose key added
            new_id = int(self.agent.poseKey)    #The posekey id that you will start searching at
            while(len(self.q_gps) > 1):       #If measurement in queue and the oldest measurment is later than current posekey
                print("new_id:%d"%new_id)
                oldest_measurement_time = (self.q_gps[0].header.stamp.sec * 1_000_000_000 + self.q_gps[0].header.stamp.nanosec) 
                print("oldest measurment time: %d\n"%oldest_measurement_time)
                next_measurement_time = (self.q_gps[1].header.stamp.sec * 1_000_000_000 + self.q_gps[1].header.stamp.nanosec ) 
                print("next measurment time: %d\n"%next_measurement_time)
                print('gps_q > 1')
                print(len(self.q_gps))
                if(oldest_measurement_time < curr_time):
                    print('oldest measurement time < curr time')
                    
                    newer_key_time = self.poseKey_to_time[int(new_id)] 
                    print("newer key time: %d\n"%newer_key_time)
                    older_key_time = self.poseKey_to_time[int(new_id - 1)]
                    print("older key time: %d\n"% older_key_time)
                    time_to_current = abs(newer_key_time - oldest_measurement_time) 
                    print("time to current: %d\n"%time_to_current)
                    time_to_previous = abs(older_key_time - oldest_measurement_time) 
                    print("time to previous: %d\n"%time_to_previous)

                    if(time_to_current > time_to_previous):
                        print('time to current is longer than time to prev')
                        new_id -= 1
                        if(new_id == self.gps_last_pose_key):
                            print('pop')
                            self.q_gps.pop(0)
                            new_id = int(self.agent.poseKey)
                    else:
                        print('time to current is shortest')

                        time_old_to_pose = abs(oldest_measurement_time - newer_key_time)
                        print("time old to pose: %d\n"%time_old_to_pose)
                        time_next_to_pose = abs(next_measurement_time - newer_key_time)
                        print("time next to pose: %d\n"%time_old_to_pose)

                        if(next_measurement_time < newer_key_time):
                        # Take care of where next measurement is not past next node
                            self.q_gps.pop(0)
                        
                        elif(time_old_to_pose > time_next_to_pose):
                        # Take care of case where next measurement is better
                            self.q_gps.pop(0)
                            new_id = self.agent.poseKey

                        else:
                            #Actually add the gps factor
                            gps_msg = self.q_gps.pop(0)

                            gps_meas = gtsam.Point3(gps_msg.pose.pose.position.x, gps_msg.pose.pose.position.y, gps_msg.pose.pose.position.z)
                            self.graph.add(gtsam.CustomFactor(self.GPS_NOISE, [new_id], partial(self.error_gps, gps_meas)))
                            self.get_logger().info("added gps unary")
                            
                            self.gps_last_pose_key = new_id
                            new_id = self.agent.poseKey
        elif sensor == 'depth' and len(self.q_depth) > 1 or sensor == 'imu' and len(self.q_imu) > 1:
            
            msg_queue = []
            if sensor == 'depth':
                # self.get_logger().info("depth unary")
                msg_queue = self.q_depth # not a copy, the reference
                last_pose_key = self.imu_last_pose_key
            elif sensor == 'imu':
                # self.get_logger().info("imu unary")
                msg_queue = self.q_imu # not a copy, the reference
                last_pose_key = self.depth_last_pose_key

            
            time_of_earliest_msg = msg_queue[0].header.stamp.nanosec + msg_queue[0].header.stamp.sec * 1e9
            keepLookingForClosest = True
            changed_last = False
            while(keepLookingForClosest):
                
                if self.poseKey_to_time.get(int(last_pose_key + 1)) is not None:
                    time_of_next_pose = self.poseKey_to_time[int(last_pose_key + 1)]
                else:
                    time_of_next_pose = None

                if time_of_next_pose is not None:
                    last_pose_key = int(last_pose_key + 1)
                    changed_last = True
                else:
                    keepLookingForClosest = False
                
            if(changed_last):
                if(self.poseKey_to_time.get(int(last_pose_key + 1)) and abs(time_of_earliest_msg - self.poseKey_to_time[int(last_pose_key)]) < abs(time_of_earliest_msg - self.poseKey_to_time[int(last_pose_key + 1)])):
                    next_oldest_measurement_msg = msg_queue.pop()
                    quat = [next_oldest_measurement_msg.orientation.x, next_oldest_measurement_msg.orientation.y, next_oldest_measurement_msg.orientation.z, next_oldest_measurement_msg.orientation.w]
                    r = R.from_quat(quat)
                    orientation_matrix = r.as_matrix()
                    # Get the orientation covariance
                    orientation_meas = gtsam.Pose3(self.HfromRT(orientation_matrix, [0,0,0])).rotation()
                    self.graph.add(gtsam.CustomFactor(self.UNARY_HEADING_NOISE, [int(self.agent.poseKey)], partial(self.error_unary_heading, [orientation_meas])))
                
            while(int(last_pose_key) < int(self.agent.poseKey)):
                time_of_pose = self.poseKey_to_time[int(last_pose_key + 1)]
                if(time_of_earliest_msg < time_of_pose):
                    right_ns = None
                    while len(msg_queue) > 1 and right_ns == None:
                        oldest_measurement_msg = msg_queue.pop() 
                        left_ns = oldest_measurement_msg.header.stamp.nanosec + oldest_measurement_msg.header.stamp.sec * 1e9
                        right_ns = msg_queue[1].header.stamp.nanosec + msg_queue[1].header.stamp.sec * 1e9 < time_of_pose
                        if right_ns < time_of_pose:
                            right_ns = None
                    if right_ns != None:
                        if (abs(left_ns - time_of_pose)) > abs(right_ns - time_of_pose):
                            next_oldest_measurement_msg = msg_queue.pop()

                            if sensor == 'depth':
                                self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [int(self.agent.poseKey)], partial(self.error_depth, next_oldest_measurement_msg.pose.pose.position.z)))
                                self.get_logger().info("added depth unary")

                            elif sensor == 'imu':
                                quat = [next_oldest_measurement_msg.orientation.x, next_oldest_measurement_msg.orientation.y, next_oldest_measurement_msg.orientation.z, next_oldest_measurement_msg.orientation.w]
                                r = R.from_quat(quat)
                                orientation_matrix = r.as_matrix()
                                # Get the orientation covariance
                                orientation_meas = gtsam.Pose3(self.HfromRT(orientation_matrix, [0,0,0])).rotation()
                                self.graph.add(gtsam.CustomFactor(self.UNARY_HEADING_NOISE, [int(self.agent.poseKey)], partial(self.error_unary_heading, [orientation_meas])))
                                self.get_logger().info("added imu unary")
                        else:
                            if sensor == 'depth':
                                self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [int(self.agent.poseKey)], partial(self.error_depth, oldest_measurement_msg.pose.pose.position.z)))
                                self.get_logger().info("added depth unary")

                            elif sensor == 'imu':
                                quat = [oldest_measurement_msg.orientation.x, oldest_measurement_msg.orientation.y, oldest_measurement_msg.orientation.z, oldest_measurement_msg.orientation.w]
                                r = R.from_quat(quat)
                                orientation_matrix = r.as_matrix()
                                # Get the orientation covariance
                                orientation_meas = gtsam.Pose3(self.HfromRT(orientation_matrix, [0,0,0])).rotation()
                                self.graph.add(gtsam.CustomFactor(self.UNARY_HEADING_NOISE, [int(self.agent.poseKey)], partial(self.error_unary_heading, [orientation_meas])))
                                self.get_logger().info("added imu unary")

                        last_pose_key = int(last_pose_key + 1)

            if sensor == 'depth':
                self.depth_last_pose_key = int(last_pose_key)
            elif sensor == 'imu':
                self.imu_last_pose_key = int(last_pose_key)



    ##################################################################
    ####################### factor graph stuff #######################
    ##################################################################
    
    def factor_graph_timer(self):
        # Your timer callback function
        # self.get_logger().info('factor_graph_timer function is called')
        if self.deployed:
            print('posekey:', self.agent.poseKey)
            r = R.from_quat(self.dvl_quat)
            self.dvl_orientation_matrix = r.as_matrix()
            self.dvl_pose_current = gtsam.Pose3(self.HfromRT(self.dvl_orientation_matrix ,self.dvl_position))
            
            # get the pose2 wrt pose1
            H_pose2_wrt_pose1_noisy = self.dvl_position_last.inverse().compose(self.dvl_pose_current)

            # add the odometry
            self.agent.prevPoseKey = int(self.agent.poseKey)
            self.agent.poseKey = int(1 + self.agent.poseKey)

            # this is the 
            self.initialEstimate.insert(self.agent.poseKey, self.dvl_pose_current)
            self.graph.add(gtsam.BetweenFactorPose3(self.agent.prevPoseKey, self.agent.poseKey, H_pose2_wrt_pose1_noisy, self.DVL_NOISE))
            self.poseKey_to_time[int(self.agent.poseKey)] = self.dvl_time
            

            # IMU unary factor
            self.unary_assignment('imu')
            print('out of imu')

            # Depth unary factor
            self.unary_assignment('depth')
            print('out of depth')

            # GPS unary factor
            self.unary_assignment('gps')
            print('out of gps')

            self.dvl_position_last = self.dvl_pose_current

            self.update()
            self.publish_vehicle_status()





    # PUBLISH THE DATA

    def publish_vehicle_status(self):
        
        # # Set the orientation in the message
        r = R.from_matrix(self.orientation_matrix)
        quat = r.as_quat()
        self.odom_msg.pose.pose.orientation.x = quat[0]
        self.odom_msg.pose.pose.orientation.y = quat[1]
        self.odom_msg.pose.pose.orientation.z = quat[2]
        self.odom_msg.pose.pose.orientation.w = quat[3]

        # # Set the position in the message
        self.odom_msg.pose.pose.position.x = self.xyz[0]
        self.odom_msg.pose.pose.position.y = self.xyz[1]
        self.odom_msg.pose.pose.position.z = self.position[2]

        # Set the covariance
        #0.2 is z covariance
        self.odom_msg.pose.covariance = [
            self.position_covariance[0, 0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.position_covariance[1, 1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.orientation_covariance[0, 0], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.orientation_covariance[1, 1], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.orientation_covariance[2, 2]
        ]


        # Publish the vehicle status
        self.vehicle_status_pub.publish(self.odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

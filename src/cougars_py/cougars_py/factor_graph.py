import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np
from functools import partial




class Agent():
    def __init__(self, H_init):
        self.pose_world_noisy = gtsam.Pose3(H_init[1])
        self.prev_pose_world_noisy_comms = self.pose_world_noisy
        self.poseKey = int((H_init[0] + 1) * 1e6)
        self.prevPoseKey = self.poseKey
        self.poseKeyStart = self.poseKey
        self.agent_num = H_init[0]
        self.path = generate_path(self.agent_num,PATH_LENGTH)
        self.step = 0
        self.x_noisy = {}
        self.y_noisy = {}


class FactorGraphNode(Node):

    def __init__(self):
        super().__init__('factor_graph_node')

        self.q_depth = []
        self.q_imu = []
        self.q_gps = []
        self.q_dvl = []
        
        
        self.gps_last_pose_key = None           #int for handling not adding more than one gps to graph
        self.depth_last_pose_key = None
        self.imu_last_pose_key = None   

        # gtsam stuff
        self.std_pose = np.array([0.01, 0.01, 0.01, np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        self.std_comms = np.array([0.01, 0.01, 0.01, np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        self.POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_pose)
        self.COMMS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_comms)


        self.isam = gtsam.ISAM2()
        self.graph = gtsam.NonlinearFactorGraph()
        self.initialEstimate = gtsam.Values()

        self.agent

        self.deployed = False

        # Initialize class variables
        self.orientation_matrix = np.eye(3)
        self.orientation_covariance = np.zeros((3, 3))
        self.position = np.zeros(3)
        self.position_covariance = np.zeros((3, 3))
        self.dvl_position = np.zeros(3)
        self.dvl_position_covariance = np.zeros((3, 3))
        self.init_state = {}

        # Subscribers
        self.create_subscription(Imu, '/modem_imu', self.imu_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/depth_data', self.depth_callback, 10)
        self.create_subscription(Odometry, '/gps_odom', self.gps_callback, 10)
        self.create_subscription(Odometry, '/dvl_dead_reckoning', self.dvl_callback, 10)
        self.create_subscription(Empty, '/init', self.init_callback, 10)

        # Publisher
        self.vehicle_status_pub = self.create_publisher(Odometry, '/vehicle_status', 10)
        
        # Timer
        self.timer = self.create_timer(1.0, self.factor_graph_timer)
    

    def get_unary_bearing(self, pose, measurement):

        rot_pred = pose.rotation()

        # Get full relative pose meas
        rot_meas = measurement[0]

        # print("rot pred", rot_pred)
        # print("rot meas", rot_meas)

        error = gtsam.Rot3.Logmap(rot_pred.between(rot_meas))
        # print("between anchor error", error)

        return error

    def error_unary_bearing(self, measurement: np.ndarray, this: gtsam.CustomFactor,
                values: gtsam.Values,
                jacobians: Optional[List[np.ndarray]]) -> float:
        
        '''Interagent factor using bearing azimuth and elevation'''

        # azimuth and elevation
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

                error_forward = get_unary_bearing(poseA_f, measurement)
                error_backward = get_unary_bearing(poseA_b, measurement)
                hdot =  (error_forward - error_backward) / (2*eps)
                H0[:,i] = hdot

            jacobians[0] = H0

        error = get_unary_bearing(pose, measurement)

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


    def HfromRT(self, R, t):
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = t
        return H

   
    def update(self):
        self.isam.update(self.graph, self.initialEstimate)
        self.result = self.isam.calculateEstimate()
        self.initialEstimate.clear()
        self.graph.resize(0)

    def imu_callback(self, msg: Imu):
        # Convert quaternion to rotation matrix
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(quat)
        self.orientation_matrix = r.as_matrix()
        # Get the orientation covariance
        self.orientation_covariance = np.array(msg.orientation_covariance).reshape(3, 3)


        if self.deployed:
            self.q_imu.append(msg)

        # dummy_t = [0,0,0]
        # orientation_meas = Pose3(self.HfromRT(self.orientation_matrix, dummy_t)).rotation()
        # # orientation 
        # self.graph.add(gtsam.CustomFactor(self.UNARY_BEARING_NOISE, [agent.poseKey], partial(self.error_unary_bearing, [orientation_meas])))

        # self.update()

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        # Set the z position and covariance

        self.position[2] = msg.pose.pose.position.z
    

        if self.deployed:
            self.q_depth.append(msg)
        # self.z_covariance = msg.pose.covariance[14]  # Covariance for Z
        # self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [agent.poseKey], partial(self.error_depth, msg.pose.pose.position.z)))

        # self.update()
                
    def gps_callback(self, msg: Odometry):
        # Get the x, y position and position covariance
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position_covariance = np.array(msg.pose.covariance).reshape(3, 3)

        if self.deployed :
            self.q_gps.append(msg)
        # gps_meas = gtsam.Point3(self.position)
        # gps_factor = gtsam.CustomFactor(GPS_NOISE, [agent.poseKey], partial(error_gps, gps_meas))
        # factor_graph.add(gps_factor)

        # self.update()

    def dvl_callback(self, msg: Odometry):
        # Get the x, y, z position
        self.dvl_position[0] = msg.pose.pose.position.x
        self.dvl_position[1] = msg.pose.pose.position.y
        self.dvl_position[2] = msg.pose.pose.position.z
        self.dvl_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.dvl_time = msg.header.stamp.nsecs

        # TODO: need covariance matrix, look into covariance, figure of merit

    


    def init_callback(self, msg: Empty):
        # Store current state as the initial state
        self.init_state = {
            'position': self.position.copy(),
            'orientation_matrix': self.orientation_matrix,
            'dvl_position': self.dvl_position.copy()
        }

        H = self.HfromRT(init_state['orientation_matrix'],init['position'])

        self.agent = Agent(H)

        self.dvl_position_last = self.agent.agent_pose_world_noisy

        priorFactor = gtsam.PriorFactorPose3(self.agent.poseKey, self.agent.pose_world_noisy, self.POSE_NOISE)
        self.graph.push_back(priorFactor)
        self.initialEstimate.insert(self.agent.poseKey, self.agent.pose_world_noisy)

    

        self.get_logger().info("Initial state has been set.")


        self.deployed = True

    def factor_graph_timer(self):
        # Your timer callback function
        # self.get_logger().info('factor_graph_timer function is called')
        if self.deployed:
            r = R.from_quat(self.dvl_quat)
            self.dvl_orientation_matrix = r.as_matrix()
            self.dvl_pose_current = gtsam.Pose3(self.HfromRT(r,self.dvl_position))
            
            # get the pose2 wrt pose1
            H_pose2_wrt_pose1_noisy = self.dvl_position_last.inverse().compose(self.dvl_pose_current)

            # add the odometry
            self.agent.prevPoseKey = int(self.agent.poseKey)
            self.agent.poseKey = int(1 + self.agent.poseKey)

            # this is the 
            self.initialEstimate.insert(self.agent.poseKey, self.agent.pose_world_noisy)

            self.graph.add(gtsam.BetweenFactorPose3(self.agent.prevPoseKey, self.agent.poseKey, H_pose2_wrt_pose1_noisy, self.POSE_NOISE))
            

            # IMU unary factor
            # FIX THIS: shouldn't be self.dvl_time - needs to be the oldest dvl node not updated
            time_of_earliest_msg = self.q_imu[0].header.stamp.nsecs
            while(time_of_earliest_msg < self.dvl_time):
                time_of_next_msg = self.q_imu[1].header.stamp.nsecs
                left = None
                while len(q_imu) > 1 and time_of_next_msg < self.dvl_time:
                    left_ns = self.q_imu[0].header.stamp.nsecs 
                    self.q_imu = self.q_imu[1:] # pop with our list implementation
                    right_ns = self.q_imu[0]
                
                if left != None:
                    if (abs(left_ns - self.dvl_time) > abs(right_ns - self.dvl_time)):































                        

                        
                    
                    




            # Depth unary factor





            # GPS unary factor
            curr_time = self.dvl_time       #Timestamp of the latest node in graph
            while(len(self.q_gps > 1) and self.q_gps[-1][1] >)












            self.dvl_position_last = self.dvl_pose_current



    def publish_vehicle_status(self):
        odom_msg = Odometry()

        # Set the orientation in the message
        r = R.from_matrix(self.orientation_matrix)
        quat = r.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set the position in the message
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.z_position

        # Set the covariance
        odom_msg.pose.covariance = [
            self.position_covariance[0, 0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.position_covariance[1, 1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, self.z_covariance, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.orientation_covariance[0, 0], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.orientation_covariance[1, 1], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.orientation_covariance[2, 2]
        ]

        # Publish the vehicle status
        self.vehicle_status_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

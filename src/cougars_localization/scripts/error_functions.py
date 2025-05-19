import gtsam
import numpy as np
from typing import List, Optional

# error functions for unary factors
def get_unary_heading(pose, measurement):

    rot_pred = pose.rotation()

    # Get full relative pose meas
    rot_meas = measurement[0]

    # print("rot pred", rot_pred)
    # print("rot meas", rot_meas)

    error = gtsam.Rot3.Logmap(rot_pred.between(rot_meas))
    # print("between anchor error", error)

    return error

def error_unary_heading(measurement: np.ndarray, this: gtsam.CustomFactor,
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

            error_forward = get_unary_heading(poseA_f, measurement)
            error_backward = get_unary_heading(poseA_b, measurement)
            hdot =  (error_forward - error_backward) / (2*eps)
            H0[:,i] = hdot

        jacobians[0] = H0

    error = get_unary_heading(pose, measurement)

    return error

def error_gps(measurement: np.ndarray, this: gtsam.CustomFactor,
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

def error_depth(measurement: np.ndarray, this: gtsam.CustomFactor,
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

def error_dvl(measurement: np.ndarray, this: gtsam.CustomFactor,
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

    # Three rows bc [x, y, z] from gps state, six columns bc the state of this node is Pose3
    
    error = vel - measurement
    
    # chat gpt helped on this part -- good to check this first
    if jacobians is not None:
        jacobians[0] = np.eye(3)
        
    return error
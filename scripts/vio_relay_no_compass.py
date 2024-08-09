#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

# Global variables to store the initial quaternion and rotation matrix for each topic
initial_quaternions = {
    'down': None,
    'right': None,
    'left': None,
    'stereo': None
}
rotation_matrices = {
    'down': None,
    'right': None,
    'left': None,
    'stereo': None
}

# Fixed goal quaternion
goal_quaternion = [0, 0, 0, 1]

def quaternion_to_yaw(quat):
    """Convert a quaternion into a yaw angle."""
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def calculate_rotation_matrix(q_init, q_goal):
    """Calculate the 3D rotation matrix to align q_init to q_goal."""
    # Convert quaternions to rotation matrices
    R_init = quaternion_matrix(q_init)[:3, :3]
    R_goal = quaternion_matrix(q_goal)[:3, :3]
    
    # Calculate the rotation matrix to transform from q_init to q_goal
    R = np.dot(R_goal, R_init.T)
    return R

def apply_rotation(pose, rotation_matrix):
    # Rotate the position
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    rotated_position = np.dot(rotation_matrix, position)
    pose.position.x = rotated_position[0]
    pose.position.y = rotated_position[1]
    pose.position.z = rotated_position[2]

    # Rotate the orientation
    q_init = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R_init = quaternion_matrix(q_init)[:3, :3]
    R_rot = np.dot(rotation_matrix, R_init)
    q_rot = quaternion_from_matrix(np.vstack((np.hstack((R_rot, [[0], [0], [0]])), [0, 0, 0, 1])))
    
    pose.orientation.x = q_rot[0]
    pose.orientation.y = q_rot[1]
    pose.orientation.z = q_rot[2]
    pose.orientation.w = q_rot[3]

    return pose

def apply_covariance_rotation(covariance, rotation_matrix):
    cov_matrix = np.array([
        [covariance[0], covariance[1], covariance[2]],
        [covariance[6], covariance[7], covariance[8]],
        [covariance[12], covariance[13], covariance[14]]
    ])
    rotated_cov_matrix = rotation_matrix.dot(cov_matrix).dot(rotation_matrix.T)
    covariance = np.array(covariance)  # Convert to numpy array for assignment
    covariance[0] = rotated_cov_matrix[0, 0]
    covariance[1] = rotated_cov_matrix[0, 1]
    covariance[2] = rotated_cov_matrix[0, 2]
    covariance[6] = rotated_cov_matrix[1, 0]
    covariance[7] = rotated_cov_matrix[1, 1]
    covariance[8] = rotated_cov_matrix[1, 2]
    covariance[12] = rotated_cov_matrix[2, 0]
    covariance[13] = rotated_cov_matrix[2, 1]
    covariance[14] = rotated_cov_matrix[2, 2]
    return covariance.tolist()  # Convert back to list

def vio_callback(data, direction):
    global initial_quaternions, rotation_matrices
    
    if initial_quaternions[direction] is None:
        q = data.pose.pose.orientation
        initial_quaternions[direction] = [q.x, q.y, q.z, q.w]  # [x, y, z, w] for tf
        rospy.loginfo("Initial quaternion recorded for %s: %s", direction, initial_quaternions[direction])
        
        # Compute the rotation matrix
        rotation_matrices[direction] = calculate_rotation_matrix(initial_quaternions[direction], goal_quaternion)
        rospy.loginfo("Rotation matrix computed for %s: \n%s", direction, rotation_matrices[direction])
    
    # Create PoseWithCovarianceStamped message
    pose_cov_msg = PoseWithCovarianceStamped()
    pose_cov_msg.header = data.header
    
    # Apply the rotation to the pose data
    if rotation_matrices[direction] is not None:
        pose_cov_msg.pose.pose = apply_rotation(data.pose.pose, rotation_matrices[direction])
        data.pose.covariance = apply_covariance_rotation(data.pose.covariance, rotation_matrices[direction])
    else:
        pose_cov_msg.pose.pose = data.pose.pose
    
    # Transfer covariance data from the incoming message
    pose_cov_msg.pose.covariance = data.pose.covariance
    
    # Publish the modified message to mavros/vision_pose/pose_cov
    vio_pub.publish(pose_cov_msg)

def vio_relay():
    rospy.init_node('vio_relay', anonymous=True)
    
    # Publisher for the MAVROS vision pose topic
    global vio_pub
    vio_pub = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=10)
    
    # Subscriber for the incoming VIO topics
    rospy.Subscriber('/down/ov_msckf/poseimu', PoseWithCovarianceStamped, vio_callback, 'down')
    rospy.Subscriber('/right/ov_msckf/poseimu', PoseWithCovarianceStamped, vio_callback, 'right')
    rospy.Subscriber('/left/ov_msckf/poseimu', PoseWithCovarianceStamped, vio_callback, 'left')
    rospy.Subscriber('/stereo/ov_msckf/poseimu', PoseWithCovarianceStamped, vio_callback, 'stereo')
    
    rospy.spin()

if __name__ == '__main__':
    try:
        vio_relay()
    except rospy.ROSInterruptException:
        pass

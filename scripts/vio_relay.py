#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from tf.transformations import euler_from_quaternion

# Global variables to store the initial quaternion, goal quaternion, and rotation matrix for each topic
initial_quaternions = {
    'down': None,
    'right': None,
    'left': None
}
goal_quaternions = {
    'down': None,
    'right': None,
    'left': None
}
rotation_matrices = {
    'down': None,
    'right': None,
    'left': None
}

def quaternion_to_yaw(quat):
    """Convert a quaternion into a yaw angle."""
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def calculate_yaw_rotation_matrix(q_init, q_goal):
    """Calculate the 2D rotation matrix to align yaw from q_init to q_goal."""
    yaw_init = quaternion_to_yaw(q_init)
    yaw_goal = quaternion_to_yaw(q_goal)
    yaw_diff = yaw_goal - yaw_init
    cos_yaw = np.cos(yaw_diff)
    sin_yaw = np.sin(yaw_diff)
    return np.array([
        [cos_yaw, -sin_yaw],
        [sin_yaw, cos_yaw]
    ])

def apply_rotation(pose, rotation_matrix):
    xy = np.array([pose.position.x, pose.position.y])
    rotated_xy = rotation_matrix.dot(xy)
    pose.position.x = rotated_xy[0]
    pose.position.y = rotated_xy[1]
    return pose

def apply_covariance_rotation(covariance, rotation_matrix):
    cov_matrix = np.array([
        [covariance[0], covariance[1]],
        [covariance[6], covariance[7]]
    ])
    rotated_cov_matrix = rotation_matrix.dot(cov_matrix).dot(rotation_matrix.T)
    covariance = np.array(covariance)  # Convert to numpy array for assignment
    covariance[0] = rotated_cov_matrix[0, 0]
    covariance[1] = rotated_cov_matrix[0, 1]
    covariance[6] = rotated_cov_matrix[1, 0]
    covariance[7] = rotated_cov_matrix[1, 1]
    return covariance.tolist()  # Convert back to list

def imu_callback(data, direction):
    global initial_quaternions, goal_quaternions, rotation_matrices
    if initial_quaternions[direction] is not None and goal_quaternions[direction] is None:
        q = data.orientation
        goal_quaternions[direction] = [q.x, q.y, q.z, q.w]  # [x, y, z, w] for tf
        rospy.loginfo("Goal quaternion recorded for %s: %s", direction, goal_quaternions[direction])
        
        # Compute the rotation matrix if the initial quaternion is already set
        if initial_quaternions[direction] is not None:
            rotation_matrices[direction] = calculate_yaw_rotation_matrix(initial_quaternions[direction], goal_quaternions[direction])
            rospy.loginfo("Rotation matrix computed for %s: \n%s", direction, rotation_matrices[direction])

def vio_callback(data, direction):
    global initial_quaternions, goal_quaternions, rotation_matrices
    
    if initial_quaternions[direction] is None:
        q = data.pose.pose.orientation
        initial_quaternions[direction] = [q.x, q.y, q.z, q.w]  # [x, y, z, w] for tf
        rospy.loginfo("Initial quaternion recorded for %s: %s", direction, initial_quaternions[direction])
        
        # Compute the rotation matrix if the goal quaternion is already set
        if goal_quaternions[direction] is not None:
            rotation_matrices[direction] = calculate_yaw_rotation_matrix(initial_quaternions[direction], goal_quaternions[direction])
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
    
    # Subscriber for the IMU topic
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback, 'down')
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback, 'right')
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback, 'left')
    
    rospy.spin()

if __name__ == '__main__':
    try:
        vio_relay()
    except rospy.ROSInterruptException:
        pass

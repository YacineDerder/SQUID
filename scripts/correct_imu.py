#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from collections import deque
import numpy as np
import time

class AccelerationCorrectorNode:
    def __init__(self):
        rospy.init_node('acceleration_corrector_node', anonymous=True)
        self.raw_accel_data = deque(maxlen=1000) # Buffer to store raw accelerometer data for 5 seconds at 200 Hz
        self.corrected_pub = rospy.Publisher('/mavros/imu/data_corrected', Imu, queue_size=10)
        rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imu_callback)
        self.mean_acceleration = None
        self.rotation_matrix = None
        self.sample_duration = rospy.get_param('~sample_duration', 5)  # Default to 5 seconds
        rospy.loginfo("Sample duration received: %s seconds", self.sample_duration)  # Print received sample duration
        self.sample_count = 0
        self.initial_wait_duration = rospy.get_param('~initial_wait_duration', 5)  # Default to 5 seconds
        rospy.loginfo("Initial wait duration received: %s seconds", self.initial_wait_duration)  # Print received initial wait duration

    def imu_callback(self, data):
        if self.sample_count >= self.initial_wait_duration * 200:
            self.raw_accel_data.append(data)
            if self.sample_count == int((self.initial_wait_duration + self.sample_duration) * 200) and not self.mean_acceleration:
                self.calibrate()

            if self.mean_acceleration is not None:
                corrected_imu = self.correct_acceleration(data)
                self.corrected_pub.publish(corrected_imu)
        self.sample_count += 1

    def calibrate(self):
        accel_x = []
        accel_y = []
        accel_z = []
        for imu_data in self.raw_accel_data:
            accel_x.append(imu_data.linear_acceleration.x)
            accel_y.append(imu_data.linear_acceleration.y)
            accel_z.append(imu_data.linear_acceleration.z)
        mean_accel_x = np.mean(accel_x)
        rospy.loginfo("mean_accel_x: %s", mean_accel_x)
        mean_accel_y = np.mean(accel_y)
        rospy.loginfo("mean_accel_y: %s", mean_accel_y)
        mean_accel_z = np.mean(accel_z)
        rospy.loginfo("mean_accel_z: %s", mean_accel_z)
        self.mean_acceleration = np.array([mean_accel_x, mean_accel_y, mean_accel_z])

        # Calculate rotation matrix
        self.calculate_rotation_matrix()

    def calculate_rotation_matrix(self):
        mean_accel = self.mean_acceleration
        norm = np.linalg.norm(mean_accel)
        
        if norm == 0:
            self.rotation_matrix = np.eye(3)
        else:
            mean_unit = mean_accel / norm
            z_unit = np.array([0, 0, 1])
            v = np.cross(mean_unit, z_unit)
            s = np.linalg.norm(v)
            c = np.dot(mean_unit, z_unit)

            # Construct the skew-symmetric cross-product matrix of v
            vx = np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])

            # Calculate the rotation matrix using the Rodrigues' rotation formula
            if s == 0:
                self.rotation_matrix = np.eye(3)
            else:
                self.rotation_matrix = np.eye(3) + vx + np.dot(vx, vx) * ((1 - c) / (s ** 2))

        # Log the rotation matrix
        rospy.loginfo("Rotation matrix calculated:")
        rospy.loginfo("\n%s\n%s\n%s", self.rotation_matrix[0], self.rotation_matrix[1], self.rotation_matrix[2])

    def correct_acceleration(self, data):
        corrected_imu = Imu()
        corrected_imu.header = data.header
        corrected_imu.orientation = data.orientation
        corrected_imu.orientation_covariance = data.orientation_covariance
        corrected_imu.angular_velocity = data.angular_velocity
        corrected_imu.angular_velocity_covariance = data.angular_velocity_covariance
        
        raw_accel = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
        corrected_accel = np.dot(self.rotation_matrix, raw_accel)
        
        corrected_imu.linear_acceleration.x = corrected_accel[0]
        corrected_imu.linear_acceleration.y = corrected_accel[1]
        corrected_imu.linear_acceleration.z = corrected_accel[2]
        corrected_imu.linear_acceleration_covariance = data.linear_acceleration_covariance
        return corrected_imu

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        acceleration_corrector = AccelerationCorrectorNode()
        acceleration_corrector.run()
    except rospy.ROSInterruptException:
        pass

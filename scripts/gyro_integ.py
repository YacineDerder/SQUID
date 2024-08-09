#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_multiply, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

class GyroIntegrationNode:
    def __init__(self):
        rospy.init_node('gyro_integration_node', anonymous=True)

        self.initial_orientation = None
        self.current_orientation = None
        self.latest_orientation = None
        self.last_update_time = None
        self.latest_reset_msg = None

        # Change subscriber to /mavros/imu/data_raw
        self.sub_imu_raw = rospy.Subscriber("/mavros/imu/data_raw", Imu, self.raw_imu_callback)
        self.sub_imu = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.sub_reset = rospy.Subscriber("/gyro/reset", Bool, self.reset_callback)
        self.pub_gyro_quaternion = rospy.Publisher("/gyro/quaternion", Quaternion, queue_size=10)

        rospy.loginfo("Gyro Integration Node Initialized.")

    def raw_imu_callback(self, msg):
        if self.initial_orientation is None:
            return

        if self.current_orientation is None:
            self.current_orientation = self.initial_orientation

        angular_velocity = msg.angular_velocity
        
        # Calculate dt based on previous orientation update time
        if self.last_update_time is None:
            self.last_update_time = rospy.Time.now()
            return
        
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time
        
        rospy.logdebug(f"dt = {dt}")

        if dt <= 0:
            return

        dq = quaternion_from_euler(angular_velocity.x * dt, angular_velocity.y * dt, angular_velocity.z * dt)

        self.current_orientation = quaternion_multiply(self.current_orientation, dq)
        self.publish_orientation_estimate()

    def imu_callback(self, msg):
        if self.initial_orientation is None:
            self.initial_orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
        else:
            self.latest_orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
        
        # Update last orientation update time
        self.last_update_time = rospy.Time.now()

    def reset_callback(self, msg):
        
        if msg.data and self.latest_orientation is not None:
            self.current_orientation = self.latest_orientation
            
            if self.latest_reset_msg is not True:
                rospy.loginfo("Orientation estimation reset.")
        self.latest_reset_msg = msg.data

    def publish_orientation_estimate(self):
        if self.current_orientation is not None:
            orientation_msg = Quaternion()
            orientation_msg.x = self.current_orientation[0]
            orientation_msg.y = self.current_orientation[1]
            orientation_msg.z = self.current_orientation[2]
            orientation_msg.w = self.current_orientation[3]
            
            # Publish the quaternion message
            self.pub_gyro_quaternion.publish(orientation_msg)

if __name__ == '__main__':
    try:
        gyro_integ_node = GyroIntegrationNode()
        rospy.spin()  # Use rospy.spin() to keep the node running and process callbacks
    except rospy.ROSInterruptException:
        pass

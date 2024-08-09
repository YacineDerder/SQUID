#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
import threading
import time

class GyroResetNode:
    def __init__(self):
        self.pub = rospy.Publisher('/gyro/reset', Bool, queue_size=10)
        self.rate = rospy.Rate(20)  # 20 Hz
        self.publish_true = True

        self.acceleration_threshold = 3 * 9.81  # 3g in m/s^2
        self.acceleration_exceeded = False

        rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imu_callback)

        # Start a thread to handle the acceleration monitoring
        self.monitor_thread = threading.Thread(target=self.monitor_acceleration)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def imu_callback(self, msg):
        acceleration = msg.linear_acceleration
        accel_magnitude = (acceleration.x**2 + acceleration.y**2 + acceleration.z**2)**0.5

        if accel_magnitude > self.acceleration_threshold:
            self.acceleration_exceeded = True

    def monitor_acceleration(self):
        while not rospy.is_shutdown():
            if self.acceleration_exceeded:
                self.publish_true = False
                time.sleep(0.5)
                self.publish_true = True
                self.acceleration_exceeded = False

    def run(self):
        while not rospy.is_shutdown():
            reset_msg = Bool(data=self.publish_true)
            self.pub.publish(reset_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gyro_reset', anonymous=True)
    node = GyroResetNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

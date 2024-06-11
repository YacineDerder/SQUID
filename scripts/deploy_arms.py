#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandLong

class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller_node', anonymous=True)
        rospy.loginfo("Servo controller node initialized.")

        # Fetching parameters or setting defaults
        self.acceleration_threshold_gs = rospy.get_param("~acceleration_threshold", 2.0)
        self.servo_high_value = rospy.get_param("~servo_high_value", 1100)
        self.servo_low_value = rospy.get_param("~servo_low_value", 1475)
        self.deploy_delay = rospy.get_param("~deploy_delay", 1)  # Default to 1 second
        self.lock_delay = rospy.get_param("~lock_delay", 1)  # Default to 1 second

        self.m_s2_to_g = 1 / 9.81  # Conversion factor from m/s^2 to G
        self.has_deployed = False
        self.command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        self.imu_subscriber = rospy.Subscriber('/mavros/imu/data_corrected', Imu, self.imu_callback)  # Subscribe to IMU data topic

        rospy.on_shutdown(self.release_servo_high)
        self.set_servo_low()  # Set servo to low value when initialized
        rospy.spin()

    def imu_callback(self, imu_msg):
        if not self.has_deployed:
            # Extract z acceleration from the IMU message and convert to G
            z_acceleration = imu_msg.linear_acceleration.z * self.m_s2_to_g
            rospy.logdebug("Z Acceleration: {:.2f} G".format(z_acceleration))
            if z_acceleration > self.acceleration_threshold_gs:
                rospy.loginfo("High acceleration detected. Waiting for {} seconds before sending servo command.".format(self.deploy_delay))
                rospy.sleep(self.deploy_delay)  # Wait for deploy_delay seconds
                rospy.loginfo("Sending servo command to set servo 5 to {} due to deploying arms.".format(self.servo_high_value))
                self.send_servo_command(self.servo_high_value, "deploying arms")
                rospy.sleep(self.lock_delay)  # Wait for lock_delay seconds
                rospy.loginfo("Sending servo command to set servo 5 to {}.".format(self.servo_low_value))
                self.send_servo_command(self.servo_low_value, "locking arms")
                self.has_deployed = True
                rospy.loginfo("Unsubscribed from IMU data topic.")
                rospy.loginfo("Node will do nothing until shutdown.")
                self.imu_subscriber.unregister()  # Unsubscribe from IMU data topic

    def send_servo_command(self, servo_value, reason):
        # Call MAVROS service to set servo 5
        try:
            response = self.command_service(broadcast=False, command=183, confirmation=0, param1=5, param2=servo_value, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)
            if response.success:
                rospy.loginfo("Servo command sent. Setting servo 5 to {} due to {}.".format(servo_value, reason))
            else:
                rospy.logwarn("Failed to send servo command: {}".format(response.result))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def set_servo_low(self):
        # Call MAVROS service to set servo 5 to low value (1500)
        self.send_servo_command(self.servo_low_value, "initialization")

    def release_servo_high(self):
        # Call MAVROS service to set servo 5 to high value (1900) when shutting down
        self.send_servo_command(self.servo_high_value, "shutdown")

if __name__ == '__main__':
    try:
        servo_controller = ServoController()
    except rospy.ROSInterruptException:
        pass

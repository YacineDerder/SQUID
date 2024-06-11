#!/usr/bin/env python

import rospy
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        self.rate = rospy.Rate(20)  # 20 Hz

        self.attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)

        self.current_state = State()

    def state_callback(self, state):
        self.current_state = state

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_service(True)
            rospy.loginfo("Vehicle armed")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", e)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_service(False)
            rospy.loginfo("Vehicle disarmed")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", e)

    def set_guided_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode_service(custom_mode='GUIDED_NOGPS')
            rospy.loginfo("Vehicle mode set to GUIDED_NOGPS")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", e)

    def send_attitude_setpoint(self, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, thrust=0.5, duration=5):
        end_time = rospy.Time.now() + rospy.Duration(duration)

        while rospy.Time.now() < end_time:
            attitude_setpoint = AttitudeTarget()
            attitude_setpoint.header = Header()
            attitude_setpoint.header.stamp = rospy.Time.now()
            attitude_setpoint.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            attitude_setpoint.body_rate.x = roll_rate
            attitude_setpoint.body_rate.y = pitch_rate
            attitude_setpoint.body_rate.z = yaw_rate
            attitude_setpoint.thrust = thrust

            attitude_setpoint.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE

            self.attitude_pub.publish(attitude_setpoint)
            self.rate.sleep()

    def run(self):
        self.set_guided_mode()
        self.arm()
        rospy.sleep(2)
        self.send_attitude_setpoint(roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0, thrust=0.2, duration=10)
        self.disarm()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

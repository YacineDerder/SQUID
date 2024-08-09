#!/usr/bin/env python

import rospy
from mavros_msgs.srv import ParamSet, ParamSetRequest, ParamSetResponse, ParamGet, ParamGetRequest, ParamGetResponse
from mavros_msgs.msg import ParamValue
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import threading

class ParamToggleNode:
    def __init__(self):
        self.param_name = 'EK3_ACC_P_NSE'
        self.values = [0.35, 100000.00]
        self.current_value_index = 0

        self.accel_pub = rospy.Publisher('/accel/noise_value', Float32, queue_size=10)
        self.gyro_pub = rospy.Publisher('/gyro/reset', Bool, queue_size=10)
        self.publish_rate = rospy.Rate(20)  # 20 Hz
        self.publish_true = True

        # Start a thread to listen for keyboard input
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Start a thread to publish the current parameter value
        self.publish_thread = threading.Thread(target=self.publish_current_value)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def wait_for_input(self):
        while True:
            input(f"Press Enter to toggle between {self.values[0]} and {self.values[1]} for {self.param_name}.\n")
            self.toggle_param()
            self.publish_true = not self.publish_true

    def toggle_param(self):
        self.current_value_index = 1 - self.current_value_index
        new_value = self.values[self.current_value_index]
        self.set_param(self.param_name, new_value)
        # self.get_param(self.param_name)

    def set_param(self, param_id, value):
        rospy.wait_for_service('/mavros/param/set')
        try:
            param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
            param_value = ParamValue()
            param_value.integer = 0  # Since it's a floating-point value
            param_value.real = value
            request = ParamSetRequest(param_id=param_id, value=param_value)
            response = param_set(request)
            if response.success:
                rospy.loginfo(f"Parameter {param_id} set to {value} successfully.")
            else:
                rospy.logerr(f"Failed to set parameter {param_id} to {value}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def get_param(self, param_id):
        rospy.wait_for_service('/mavros/param/get')
        try:
            param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            request = ParamGetRequest(param_id=param_id)
            response = param_get(request)
            if response.success:
                # rospy.loginfo(f"Parameter {param_id} value is {response.value.real}.")
                return response.value.real
            else:
                rospy.logerr(f"Failed to get parameter {param_id}.")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def publish_current_value(self):
        while not rospy.is_shutdown():
            current_value = self.get_param(self.param_name)
            if current_value is not None:
                self.accel_pub.publish(Float32(current_value))
                self.gyro_pub.publish(Bool(data=self.publish_true))
            self.publish_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('param_toggle_node', anonymous=True)
    node = ParamToggleNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

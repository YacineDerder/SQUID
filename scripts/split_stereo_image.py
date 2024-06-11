#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def split_stereo_image():
    rospy.init_node('stereo_image_splitter', anonymous=True)

    # Get parameters with defaults
    frame_rate = rospy.get_param('~frame_rate', 30)
    video_width = rospy.get_param('~video_width', 640)
    video_height = rospy.get_param('~video_height', 240)
    video_device = rospy.get_param('~video_device', '/dev/video6')
    
    rospy.loginfo("Frame rate set to: %d", frame_rate)
    rospy.loginfo("Video resolution set to: %dx%d", video_width, video_height)
    rospy.loginfo("Video device set to: %s", video_device)
    
    # Publishers for the left and right images
    left_pub = rospy.Publisher('/stereo_left/image_raw', Image, queue_size=10)
    right_pub = rospy.Publisher('/stereo_right/image_raw', Image, queue_size=10)
    
    bridge = CvBridge()

    # Open the video device
    cap = cv2.VideoCapture(video_device)
    if not cap.isOpened():
        rospy.logerr("Failed to open video device %s.", video_device)
        return

    rospy.loginfo("Successfully opened video device %s.", video_device)
    
    # Set the desired frame width and height
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)
    rospy.loginfo("Set video resolution to %dx%d.", video_width, video_height)

    rate = rospy.Rate(frame_rate)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to read frame from video device.")
            rate.sleep()
            continue

        try:
            timestamp = rospy.Time.now()
    
            height, width, _ = frame.shape
            mid = width // 2

            left_image = frame[:, :mid]
            right_image = frame[:, mid:]

            left_msg = bridge.cv2_to_imgmsg(left_image, "bgr8")
            right_msg = bridge.cv2_to_imgmsg(right_image, "bgr8")

            left_msg.header.stamp = timestamp
            right_msg.header.stamp = timestamp

            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: %s", str(e))
        except Exception as e:
            rospy.logerr("Unexpected error: %s", str(e))

        rate.sleep()

    cap.release()
    rospy.loginfo("Released video device.")

if __name__ == '__main__':
    try:
        split_stereo_image()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

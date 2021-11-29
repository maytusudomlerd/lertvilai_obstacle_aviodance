#!/usr/bin/env python2.7

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, msg.encoding)

        # print('Convert')
        # cv2.imshow('camera_image', cv2_img)
        # cv2.waitKey(0)
        return cv2_image
    except Exception as e:
        print(e)


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "camera/color/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
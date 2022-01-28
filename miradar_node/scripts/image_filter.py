#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()
image_pub = rospy.Publisher("/miradar/bin_image", Image)

def imagecb(data):
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    kernel = np.ones((5,5),np.uint8)
    #img = cv2.dilate(img, kernel, iterations=3)
    ret,thres= cv2.threshold(img,150,255,cv2.THRESH_BINARY)
    #thres = cv2.erode(thres, kernel, iterations=1)
    #thres = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY,11,2)
    try:
      image_pub.publish(bridge.cv2_to_imgmsg(thres, "mono8"))
    except CvBridgeError as e:
      print(e)


if __name__ == "__main__":
    rospy.init_node('radar_image_filter', anonymous=True)

    rospy.Subscriber("/miradar/image_raw", Image, imagecb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
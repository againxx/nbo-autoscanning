#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from math import isnan
import numpy as np

imageData = Image()


def imageCallback(data):
    global imageData
    imageData = data


def listener():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image,
                     imageCallback)

    rospy.sleep(0.5)

    global imageData

    bridge = CvBridge()
    cvImage = bridge.imgmsg_to_cv2(imageData, desired_encoding="passthrough")
    cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB)

    cv2.imwrite("image" + sys.argv[1] + ".jpg", cvImage)


if __name__ == "__main__":
    listener()

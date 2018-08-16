#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImagePublisher(object):
    def __init__(self, dataPath, frameName):
        self.__dataPath__ = dataPath
        self.__frameName__ = frameName

        self.__depthImage__ = cv2.imread(
            self.__dataPath__ + "depth/" + self.__frameName__ + "_depth.png",
            cv2.CV_LOAD_IMAGE_UNCHANGED)

        self.__rgbImage__ = cv2.imread(
            self.__dataPath__ + "rgb/" + self.__frameName__ + "_rgb.png",
            cv2.CV_LOAD_IMAGE_UNCHANGED)
        
        self.__loopNum__ = 0
        self.__repetitions__ = 30
        self.__counter__ = 1
        self.__unitPixels__ = 40
        self.__leftPos__ = 0
        self.__rightPos__ = 15
        self.__upPos__ = 0
        self.__downPos__ = 11
        self.__rightBound__ = self.__depthImage__.shape[0] / self.__unitPixels__ - 1
        self.__downBound__ = self.__depthImage__.shape[1] / self.__unitPixels__ - 1

    def getNextImage(self):
        truncDepthImage = self.__depthImage__[
                self.__upPos__ * self.__unitPixels__ : (self.__downPos__ + 1) * self.__unitPixels__,
                self.__leftPos__ * self.__unitPixels__ : (self.__rightPos__ + 1) * self.__unitPixels__]

        truncRGBImage = self.__rgbImage__[
                self.__upPos__ * self.__unitPixels__ : (self.__downPos__ + 1) * self.__unitPixels__,
                self.__leftPos__ * self.__unitPixels__ : (self.__rightPos__ + 1) * self.__unitPixels__]

        if self.__counter__ % self.__repetitions__ == 0:
            self.moveWindow()
        self.__counter__ += 1
        #return truncRGBImage, truncDepthImage
        return self.__rgbImage__, self.__depthImage__

    def moveWindow(self):
        if self.__upPos__ == 0 and self.__rightPos__ < self.__rightBound__:
            self.__leftPos__ += 1
            self.__rightPos__ += 1
        elif self.__rightPos__ == self.__rightBound__ and self.__downPos__ < self.__downBound__:
            self.__upPos__ += 1
            self.__downPos__ += 1
        elif self.__downPos__ == self.__downBound__ and self.__leftPos__ > 0:
            self.__leftPos__ -= 1
            self.__rightPos__ -= 1
        elif self.__leftPos__ == 0 and self.__upPos__ > 0:
            self.__upPos__ -= 1
            self.__downPos__ -= 1

    def printCurrentPos(self):
        print "Left:%d Right:%d Up:%d Down:%d" % (
                self.__leftPos__,
                self.__rightPos__,
                self.__upPos__,
                self.__downPos__
                )


def main():
    print "Begin publishing image data..."
    imagePublisher = ImagePublisher(
            "/media/vcc/DATA/3d_semantics/area_3/data/",
            "camera_4a7bfe0577f74a1a891683cf5b435f93_lounge_1_frame_21_domain")

    rgbPublisher = rospy.Publisher("/camera/single_rgb", Image, queue_size=10)
    depthPublisher = rospy.Publisher("/camera/single_depth", Image, queue_size=10)
    rospy.init_node("image_publisher", anonymous=True)
    rate = rospy.Rate(30)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        rgbImage, depthImage = imagePublisher.getNextImage()
        try:
            depthMsg = bridge.cv2_to_imgmsg(depthImage, encoding="passthrough")
            rgbMsg = bridge.cv2_to_imgmsg(rgbImage, encoding="passthrough")
        except CvBridgeError as e:
            print e

        rgbPublisher.publish(rgbMsg)
        depthPublisher.publish(depthMsg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image as msg_Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.pub = rospy.Publisher('/FindPerson', String, queue_size=1)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
            sys.stdout.flush()
            self.pub.publish(str(cv_image[pix[1], pix[0]]))
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    rospy.init_node("depth_image_processor")
    #topic = '/camera/aligned_depth_to_color/image_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
    topic = '/camera/depth/image_rect_raw'
    listener = ImageListener(topic)
    rospy.spin()
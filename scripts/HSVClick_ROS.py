#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

class HSVClick(object):
    
    def __init__(self):
        self.ImageSub = rospy.Subscriber("/camera/color/image_raw", Image, self.process_image)
        self.cvbridge = CvBridge()
        self.upper = np.array([120, 255, 255])
        self.lower = np.array([100, 200, 90])

        self.u1 = np.array([120, 255, 255])
        self.l1 = np.array([100, 200, 90])

        self.num = 10
        # ウィンドウのサイズを変更可能にする
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        # マウスイベント時に関数mouse_eventの処理を行う
        cv2.setMouseCallback("img", self.mouse_event)
        
    def mouse_event(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            self.upper[0] = self.hsv[y, x, 0] + self.num
            self.lower[0] = self.hsv[y, x, 0] - self.num

            self.upper[1] = self.hsv[y, x, 1] + self.num
            self.lower[1] = self.hsv[y, x, 1] - self.num

            self.upper[2] = self.hsv[y, x, 2] + self.num
            self.lower[2] = self.hsv[y, x, 2] - self.num
            if self.upper[0] > 180:
                self.u1[0] = self.upper[0] - 180
                self.l1[0] = 0
            elif self.lower[0] < 0:
                self.u1[0] = 180
                self.l1[0] = 180 + self.lower[0]
            else:
                self.u1[0] = 0
                self.l1[0] = 0
            print("H = ", self.upper[0] - self.num)
            print("S = ", self.upper[1] - self.num)
            print("V = ", self.upper[2] - self.num)
    def process_image(self, msg):
        try:
            img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8") 
            img = cv2.resize(img, dsize=None, fx=0.5, fy=0.5)
            height, width = img.shape[:2]
            # HSVに変換する
            self.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # マスク処理
            mask = cv2.inRange(self.hsv, self.lower, self.upper)
            mask1 = cv2.inRange(self.hsv, self.l1, self.u1)

            mask = mask + mask1
            cnts,ret = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            if len(cnts) != 0:
                cnts = max(cnts,key=lambda x: cv2.contourArea(x))
                x,y,w,h = cv2.boundingRect(cnts)
                print("Square = ",w*h)

            cv2.imshow("img", img)
            cv2.imshow("mask", mask)
            cv2.waitKey(1)
        except Exception as err:
            print(err)

def main():
    rospy.init_node('HSVCLICK')
    yoros = HSVClick()
    rospy.spin()# 色フィルター設定
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
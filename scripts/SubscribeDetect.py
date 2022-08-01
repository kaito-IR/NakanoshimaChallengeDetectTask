#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
import sys
from std_msgs.msg import Int16
from darknet_ros_msgs.msg import BoundingBoxes 
from sensor_msgs.msg import Image,CompressedImage  
from cv_bridge import CvBridge #cvBridgeはpython3では使えないっぽい

class YOROS(object):
    def __init__(self):
        self.YOLOSub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, self.Callback)
        self.ImageSub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.process_image)
        self.DepthSub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.process_depth_image)
        self.StaffDistPub = rospy.Publisher('/StaffDist', Int16, queue_size=1)
        self.PersonDistPub = rospy.Publisher('/PersonDist', Int16, queue_size=1)
        self.cvbridge = CvBridge()
        self.StaffDistOld = 0
        
    def process_image(self, msg):
        try:
            #self.img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = self.cvbridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        except Exception as err:
            print(err)
    def process_depth_image(self, msg):
        try:
            self.depthimg = self.cvbridge.imgmsg_to_cv2(msg, msg.encoding)
            #pix = (msg.width/2, msg.height/2)
            #sys.stdout.write('Depth at center(%d, %d): %f(mm)\r' % (pix[0], pix[1], self.depthimg[pix[1], pix[0]]))
            #sys.stdout.flush()
            #self.StaffDistPub.publish(str(self.depthimg[pix[1], pix[0]]))
        except Exception as err:
            print(err)

    def Callback(self, msg):
        bboxs = msg.bounding_boxes
        if len(bboxs) > 0:
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV) #BGRをHSV色空間に変換
            orange_min = np.array([5,220,200]) #多分この辺の範囲のはず．要現地で調整．
            orange_max = np.array([20,255,255])
            blue_min = np.array([100,220,80])
            blue_max = np.array([115,255,255])
            #orange_min = np.array([60,200,90]) #テスト用の範囲．本番は上の範囲でやる
            #orange_max = np.array([100,255,160])
            #blue_min = np.array([80,190,120])
            #blue_max = np.array([120,250,220])
            mask1 = cv2.inRange(hsv, blue_min, blue_max) #hsvの各ドットについてblue_minからblue_maxの範囲内ならtrue
            mask2 = cv2.inRange(hsv,orange_min,orange_max)
            mask = cv2.bitwise_or(mask1,mask2)
            mask = cv2.medianBlur(mask, 3)
            #領域のカタマリである「ブロブ」を識別し、データを格納する。すごくありがたい機能。
            nLabels, labelimages, stats, center = cv2.connectedComponentsWithStats(mask)
            blob_count = nLabels - 1 #ブロブの数。画面領域全体を1つのブロブとしてカウントするので、-1する。
            iList = []
            CenterList = []
            if blob_count > 0:
                for i in range(1, nLabels):
                    if stats[i][4] >= 300:
                        iList.append(i)
                if len(iList) != 0:
                    for i in iList:
                        CenterPos = (int(center[i][0]),int(center[i][1]))
                        CenterList.append(CenterPos)
                        cv2.circle(self.img,CenterPos,2,(0,0,255),5)
                    for i, bb in enumerate(bboxs):
                        flag = False
                        if bboxs[i].Class == 'person' and bboxs[i].probability >= 0.35:
                            for pos in CenterList:
                                if pos[0] >= bboxs[i].xmin and pos[0] <= bboxs[i].xmax and pos[1] >= bboxs[i].ymin and pos[1] <= bboxs[i].ymax:
                                    dist = int(self.depthimg[pos[1],pos[0]])
                                    #if dist > 0 and (self.StaffDistOld == 0 or abs(self.StaffDistOld - dist) <= 200):
                                    if dist > 0:
                                        self.StaffDistOld = dist
                                        self.StaffDistPub.publish(dist)#単位：mm
                                        flag = True
                                        break
                                    elif dist > 0:
                                        self.StaffDistOld = dist
                            if flag:
                                break
                else:
                    self.StaffDistOld = 0
                    for i, bb in enumerate(bboxs):
                        flag = False
                        if bboxs[i].Class == 'person' and bboxs[i].probability >= 0.35:
                            pos = (int((bboxs[i].xmax+bboxs[i].xmin)/2),int((bboxs[i].ymax+bboxs[i].ymin)/2))
                            dist = int(self.depthimg[pos[1],pos[0]])
                            if dist > 0:
                                self.PersonDistPub.publish(dist)
                                break
            else:
                self.StaffDistOld = 0
                for i, bb in enumerate(bboxs):
                    flag = False
                    if bboxs[i].Class == 'person' and bboxs[i].probability >= 0.35:
                        pos = (int((bboxs[i].xmax+bboxs[i].xmin)/2),int((bboxs[i].ymax+bboxs[i].ymin)/2))
                        dist = int(self.depthimg[pos[1],pos[0]])
                        if dist > 0:
                            self.PersonDistPub.publish(dist)
                            break
                
            #cv2.imshow("camera",self.img)
            cv2.imshow("mask",mask)
            cv2.waitKey(1)
def main():
    rospy.init_node('YOLO')
    yoros = YOROS()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
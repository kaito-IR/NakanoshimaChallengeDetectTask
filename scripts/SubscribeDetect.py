#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes 
from sensor_msgs.msg import Image,CompressedImage  
from cv_bridge import CvBridge

class YOROS(object):
    def __init__(self):
        self.YOLOSub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes, self.Callback)
        self.ImageSub = rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.process_image)
        self.DepthSub = rospy.Subscriber("/camera/depth/image_rect_raw/compressed", CompressedImage, self.process_depth_image)
        self.pub = rospy.Publisher('/FindPerson', String, queue_size=1)
        self.cvbridge = CvBridge()
        
    def process_image(self, msg):
        try:
            #self.img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = self.cvbridge.compressed_imgmsg_to_cv2(msg,"bgr8")
        except Exception as err:
            print(err)
    def process_depth_image(self, msg):
        try:
            #self.img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
            self.depthimg = self.cvbridge.compressed_imgmsg_to_cv2(msg,msg.encoding)
        except Exception as err:
            print(err)

    def Callback(self, msg):
        bboxs = msg.bounding_boxes
        if len(bboxs) > 0:
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV) #BGRをHSV色空間に変換
            #orange_min = np.array([8,200,200]) #多分この辺の範囲のはず．要現地で調整．
            #orange_max = np.array([20,255,255])
            #blue_min = np.array([110,220,130])
            #blue_max = np.array([120,255,255])
            orange_min = np.array([60,200,90]) #テスト用の範囲．本番は上の範囲でやる
            orange_max = np.array([100,255,160])
            blue_min = np.array([80,190,120])
            blue_max = np.array([120,250,220])
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
                    if stats[i][4] >= 500:
                        iList.append(i)
                for i in iList:
                    CenterPos = (int(center[i][0]),int(center[i][1]))
                    CenterList.append(CenterPos)
                    cv2.circle(self.img,CenterPos,2,(0,0,255),5)
                for i, bb in enumerate(bboxs):
                    if bboxs[i].Class == 'person' and bboxs[i].probability >= 0.40:
                        for pos in CenterList:
                            if pos[0] >= bboxs[i].xmin and pos[0] <= bboxs[i].xmax and pos[1] >= bboxs[i].ymin and pos[1] <= bboxs[i].ymax:
                                
                                indices = np.array(np.where(self.depthimg == self.depthimg[self.depthimg > 0].min()))[:,0]
                                pix = (indices[1], indices[0])
                                #self.RosFindPersonPublish(pos)
                                self.RosFindPersonPublish(self.depthimg[pix[1], pix[0]])
            #cv2.imshow("",self.img)
            cv2.imshow("mask",mask)
            #self.depthimg = cv2.cvtColor(self.depthimg,cv2.COLOR_BGR2RGB)hoge
            #cv2.imshow("depth",self.depthimg)
            cv2.waitKey(1)
    def RosFindPersonPublish(self,msg):
        self.pub.publish(str(msg))
def main():
    rospy.init_node('YOLO')
    yoros = YOROS()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
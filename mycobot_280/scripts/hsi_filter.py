#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self, node_name):
        self.node_name = node_name
        self.image_pub = rospy.Publisher(node_name, Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_rect_color",Image,self.callback)

    def callback(self,data):
        try:
            # ROSのsensor_msgs/Image型からOpenCVで処理適量にcv::Mat型へ変換する。
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # RGB表色系からHSV表色系に変換
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # しきい値の設定（ここでは赤を抽出）
        color_min = np.array([150,100, 50])
        color_max = np.array([180,255,255])

        # マスク画像を生成
        color_mask = cv2.inRange(hsv_image, color_min, color_max);
        # 画像配列のビット毎の倫理積席。マスク画像だけが抽出される。
        cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = color_mask)

        # 重心を求める。moments関数を使うためグレースケール画像へ変換。
        gray_image2 = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2GRAY)
        mu = cv2.moments(gray_image2, True)
        if mu["m00"] == 0: # マスクをかけた画像がない場合(ここでは赤）の処理
            x, y = -999, -999
        else:
            x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])

        # 重心を中心として半径20ピックセルの円を描画
        color  = (0, 255, 0)
        center = (x, y)
        radius = 20
        cv2.circle(cv_image2, center, radius, color)

        # エッジを検出する。Canny関数を使うためRGBからグレースケールへ変換
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image3  = cv2.Canny(gray_image, 15.0, 30.0);


        # ウインドウのサイズを変更
        cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)
        cv_half_image2 = cv2.resize(cv_image2, (0,0),fx=0.5,fy=0.5);
        cv_half_image3 = cv2.resize(cv_image3, (0,0),fx=0.5,fy=0.5);

        # ウインドウ表示
        cv2.imshow("Origin Image", cv_half_image)
        cv2.imshow("Result Image", cv_half_image2)
        cv2.imshow("Edge Image",   cv_half_image3)
        cv2.waitKey(3)

        try:
            # パブリッシュするためにOpenCVのcv::MatからROSのsensor_msgs/Image型へ変換
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':   
    try:
        node_name = 'camera' 
        rospy.init_node(node_name, anonymous=True)
        ImageConverter(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import os
import sys
import time

import numpy as np
import onnxruntime

import rospy
import rospkg

from sensor_msgs.msg import Image 

# OpenCV import for python3
if os.environ['ROS_PYTHON_VERSION'] == '3':
    import cv2
else:
    sys.path.remove('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA
    import cv2  # NOQA
    sys.path.append('/opt/ros/{}/lib/python2.7/dist-packages'.format(os.getenv('ROS_DISTRO')))  # NOQA

# cv_bridge_python3 import
if os.environ['ROS_PYTHON_VERSION'] == '3':
    from cv_bridge import CvBridge
else:
    ws_python3_paths = [p for p in sys.path if 'devel/lib/python3' in p]
    if len(ws_python3_paths) == 0:
        # search cv_bridge in workspace and append
        ws_python2_paths = [
            p for p in sys.path if 'devel/lib/python2.7' in p]
        for ws_python2_path in ws_python2_paths:
            ws_python3_path = ws_python2_path.replace('python2.7', 'python3')
            if os.path.exists(os.path.join(ws_python3_path, 'cv_bridge')):
                ws_python3_paths.append(ws_python3_path)
        if len(ws_python3_paths) == 0:
            opt_python3_path = '/opt/ros/{}/lib/python3/dist-packages'.format(
                os.getenv('ROS_DISTRO'))
            sys.path = [opt_python3_path] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(opt_python3_path)
        else:
            sys.path = [ws_python3_paths[0]] + sys.path
            from cv_bridge import CvBridge
            sys.path.remove(ws_python3_paths[0])
    else:
        from cv_bridge import CvBridge


class HairSegmentationNode(object):
    def __init__(self):
        self.bridge= CvBridge()
        self.rospack = rospkg.RosPack()
        self.rate = rospy.Rate(30)

        model_path = rospy.get_param(
            '~model_path',
            self.rospack.get_path('mycobot_280') + \
            '/model/DeepLabV3Plus(timm-mobilenetv3_small_100)_452_2.16M_0.8385/best_model_simplifier.onnx')
        self.score_thresh = rospy.get_param('~score_thresh', 0.3)
        self.input_size = rospy.get_param('~input_size', 512)
        
        EP_list = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.onnx_session = onnxruntime.InferenceSession(model_path, providers=EP_list)
        self.onnx_session.set_providers(['CUDAExecutionProvider'])
        self.pub = rospy.Publisher("~output", Image, queue_size=1)
        self.pub_debug = rospy.Publisher("~output/debug", Image, queue_size=1)
        self.pub_masked = rospy.Publisher("~output/masked", Image, queue_size=1)
        self.pub_contour = rospy.Publisher("~output/contour", Image, queue_size=1)
        self.pub_crown = rospy.Publisher("~output/crown", Image, queue_size=1)
               
        self.sub = rospy.Subscriber(
            "~input", Image, self.process_image)
        self.kernel_e = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        self.kernel_d = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))

        rospy.loginfo('hair segmentation node started')

    def process_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as err:
            rospy.logerr(err)
            return

        ret = len(img)
        if ret == 0:
            return

        start_time = time.time()
        masks = self.run_inference(
            img,
            self.onnx_session,
            self.input_size,
        )
        masks = np.where(masks > self.score_thresh, 0, 1)
        elapsed_time = time.time() - start_time

        mask_partial = np.where(masks[2]==0, 1, 0).astype(np.uint8)
        mask_partial *= 255
        mask_partial = cv2.resize(mask_partial, dsize=img.shape[:2][::-1])
        cnt = self.find_contours(mask_partial)

        hair_mask = self.find_hair_mask(mask_partial, cnt)
        hair_mask = cv2.morphologyEx(hair_mask, cv2.MORPH_DILATE, self.kernel_d)
        hair_mask = cv2.morphologyEx(hair_mask, cv2.MORPH_ERODE, self.kernel_e)

        debug_img = np.zeros(img.shape, dtype=np.uint8)
        debug_img[np.where(hair_mask > 0)] = np.array([0, 0, 255]) 
        debug_img = (img * 0.5 + debug_img * 0.5).astype(np.uint8)
        
        masked_img = np.zeros(img.shape, dtype=np.uint8)
        masked_img[np.where(hair_mask> 0)]= img[np.where(hair_mask>0)]
        #masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)

        mask_crown, img_crown = self.detect_crown(masked_img)
        hair_mask -= mask_crown

        contour_img = np.zeros(img.shape, dtype=np.uint8)      
        contour_img = self.draw_contours(contour_img, cnt)
        rect_img = self.draw_rect_hair(contour_img, cnt)
        
        img_msg = self.bridge.cv2_to_imgmsg(hair_mask, encoding="mono8")
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        masked_msg = self.bridge.cv2_to_imgmsg(masked_img, encoding="bgr8")
        contour_msg = self.bridge.cv2_to_imgmsg(rect_img, encoding="bgr8")
        crown_msg  =self.bridge.cv2_to_imgmsg(mask_crown, encoding="mono8")
        
        self.pub.publish(img_msg)
        self.pub_debug.publish(debug_msg)
        self.pub_masked.publish(masked_msg)
        self.pub_crown.publish(crown_msg)
        self.pub_contour.publish(contour_msg)

    def run_inference(self, image, onnx_session, input_size):
        # 前処理
        input_image = cv2.resize(image, dsize=(input_size, input_size))  # リサイズ
        input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)  # BGR→RGB変換
   
        # 標準化
        mean = [0.485, 0.456, 0.406]
        std = [0.229, 0.224, 0.225]
        x = (input_image / 255 - mean) / std
    
        # HWC → CHW
        x = x.transpose(2, 0, 1).astype('float32')
    
        # (1, 3, Height, Width)形式へリシェイプ
        x = x.reshape(-1, 3, 512, 512)
    
        # 推論
        input_name = onnx_session.get_inputs()[0].name
        output_name = onnx_session.get_outputs()[0].name
        onnx_result = onnx_session.run([output_name], {input_name: x})
        onnx_result = np.array(onnx_result).squeeze()
    
        return onnx_result

    def detect_crown(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        # hsv_min = np.array([14, 196, 89])
        # hsv_max = np.array([60, 255, 147])
        hsv_min = np.array([15, 80, 80])
        hsv_max = np.array([60, 150, 150])

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (51, 51))
        
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)
        
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        return mask, masked_img

    def find_contours(self, img):
        contours, hierarchy = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hair_contours=[]
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > 8000:
                hair_contours.append(cnt)
            # print(f"contours[{i}].shape: {cnt.shape}") 
        return hair_contours
    
    def draw_contours(self, img, cnts):
        img_ = img.copy()
        for cnt in cnts:
            cnt = cnt.squeeze(axis=1)
            # print(cnt.shape)
            for (x, y) in cnt:
                cv2.circle(img_, (x, y), 1, (0,150,150), 3)
        return img_
    
    def draw_rect_hair(self, img, cnts):
        img_ = img.copy()
        for cnt in cnts:
            x,y,w,h = cv2.boundingRect(cnt)
            img_ = cv2.rectangle(img_, (x,y),(x+w,y+h), (100, 100, 0), 2)
        return img_
    
    def find_hair_mask(self, mask, cnts):
        mask_ = mask.copy()
        mask_hair_ = np.zeros_like(mask_).astype(np.uint8)
        for cnt in cnts:
            x,y,w,h = cv2.boundingRect(cnt)
            mask_hair_[y:y+h,x:x+w]= mask_[y:y+h, x:x+w]
        
        return mask_hair_


if __name__ == '__main__':
    rospy.init_node('hair_segmentation_node')
    app = HairSegmentationNode()
    rospy.spin()

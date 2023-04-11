#!/usr/bin/env python
# coding: UTF-8#!/usr/env/bin python3

import cv2
import numpy as np
from math import pi
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import rosparam
from scipy.ndimage.filters import gaussian_filter1d
import colorsys

class PlotData:
    def __init__(self, data1, data2, data3, filtered=False):
        data1 = cv2.calcHist([data1],[0],None,[256],[-128,127])
        data2 = cv2.calcHist([data2],[0],None,[256],[0,256])
        data3 = cv2.calcHist([data3],[0],None,[256],[0,256])
        # print(data1)
        # print(data1.shape)

        self.data1 = data1
        self.data2 = data2
        self.data3 = data3
        self.filtered = filtered
    def plot(self):
        fig, ax = plt.subplots()


        ax.plot(self.data1, label='h')
        ax.plot(self.data2, label='s')
        ax.plot(self.data3, label='i')
        plt.title("hsi")

        # if self.filtered == True:
        #     ax.plot(h_smooth, label='h_smooth')
        #     ax.plot(s_smooth, label='s_smooth')
        #     ax.plot(i_smooth, label='i_smooth')

        #     plt.title("denoised")
        ax.legend()
        plt.show()


class HsiFilter:
    def __init__(self, topic_name):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.callback)
        self.hsi_filter = None
        self.sigma = 2 #for gaussian filter

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            print(e)
            return

        roi = cv2.selectROI("Image", img, fromCenter=False, showCrosshair=True)
        cv2.destroyAllWindows()

        self.hsi_filter = self.get_hsi_filter_values(img, roi)

        print(self.hsi_filter)
        print("HSI Filter values: H={:.2f}, S^2={:.2f}, I={:.2f}".format(*self.hsi_filter))

    @staticmethod
    def bgr_to_hsi(img):
        with np.errstate(divide='ignore', invalid='ignore'):
            bgr = np.int32(cv2.split(img))
            blue = bgr[0]
            green = bgr[1]
            red = bgr[2]
            print("b",blue)
            print("g", green)
            intensity = np.divide(blue + green + red, 3)
            minimum = np.minimum(np.minimum(red, green), blue)
            saturation = 1 - 3 * np.divide(minimum, red + green + blue)
            sqrt_calc = np.sqrt(((red - green) * (red - green)) + ((red - blue) * (green - blue)))
            if (green >= blue).any():
                hue = np.arccos((1/2 * ((red-green) + (red - blue)) / sqrt_calc))
                print("green !")
            else:
                hue = 2*pi - np.arccos((1/2 * ((red-green) + (red - blue)) / sqrt_calc))

            hue = hue*180/pi
            hsi = cv2.merge((hue, saturation,intensity))
            return hsi

    @staticmethod
    def denoise(sigma, data):
        result =[]
        for d in data:
            result.append(gaussian_filter1d(d, sigma))
        return result[0], result[1], result[2]

    def get_hsi_filter_values(self, img, roi):
        # print(img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsi = self.bgr_to_hsi(img)
        # print(img_hsi.shape)
        x1 = roi[0]
        y1 = roi[1]
        x2 = roi[2]
        y2 = roi[3]
        roi_hsv = img_hsv[y1:y1+y2,x1:x1+x2,:]
        roi_hsi =img_hsi[y1:y1+y2,x1:x1+x2,:]
        print("hsi h", roi_hsi[:,:,0])
        print("hsv h", roi_hsv[:,:,0])
        # roi_hsi = roi_hsi[~np.isnan(roi_hsi)]
        h = np.interp(roi_hsv[:,:,0],[0, 180], [-128, 127]).flatten() -128 # bug need -128 when blue or yellow but green is correct when no-128
        s = np.interp(roi_hsi[:,:,1],[0, 1], [0, 255]).flatten()
        i = np.interp(roi_hsi[:,:,2],[0, 255], [0,255]).flatten()
        # h = np.sort(h)
        # s = np.sort(s)
        # i = np.sort(i)
        # print(h)
        # print(s.shape)
        # print(i.shape)
        # Plot the data
        # plotter = PlotData(h, s, i, True)
        # plotter.plot()

        h, s, i = self.denoise(1, [h, s, i])
        h = h.astype(np.int8)
        s = s.astype(np.uint8)
        i = i.astype(np.uint8)
        # plotter_ = PlotData(h, s, i, True)
        # plotter_.plot()
        h_avg = np.nanmean(h)
        s_avg = np.nanmean(s)
        i_avg = np.nanmean(i)

        h_max = np.nanmax(h)
        s_max = np.nanmax(s)
        i_max = np.nanmax(i)
        h_min = np.nanmin(h)
        s_min = np.nanmin(s)
        i_min = np.nanmin(i)

        print("result:", [h_max, h_min, s_max, s_min, i_max, i_min])

        rospy.set_param("/hsi_color_filter/hsi_filter/h_limit_max", int(h_max))
        rospy.set_param("/hsi_color_filter/hsi_filter/h_limit_min", int(h_min))
        rospy.set_param("/hsi_color_filter/hsi_filter/s_limit_max", int(s_max))
        rospy.set_param("/hsi_color_filter/hsi_filter/s_limit_min", int(s_min))
        rospy.set_param("/hsi_color_filter/hsi_filter/i_limit_max", int(i_max))
        rospy.set_param("/hsi_color_filter/hsi_filter/i_limit_min", int(i_min))
        # Calculate the HSI filter values

        hsi_filter = [h_avg, s_avg, i_avg]
        return hsi_filter

if __name__ == '__main__':
    rospy.init_node('hsi_filter_node', anonymous=True)
    hsi_filter = HsiFilter("/camera/color/image_rect_color")
    rospy.spin()

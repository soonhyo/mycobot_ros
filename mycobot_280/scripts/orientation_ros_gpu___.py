#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import copy

import rospy
import rospkg
import message_filters

from sensor_msgs.msg import Image

# from std_msgs.msg import Bool
from std_msgs.msg import Int32, Int8, Int16MultiArray
from sensor_msgs.msg import PointCloud2
from mycobot_280.msg import msg_std
import sensor_msgs.point_cloud2 as pc2
from converter import numpy2i16multi 

import math
import numpy as np
import sample_cef as coh
import scipy.misc
from scipy import ndimage
from numba import cuda

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

@cuda.jit
def calculate_angles(im, result, W, Gy_, Gx_):
    """
    anisotropy orientation estimate, based on equations 5 from:
    https://pdfs.semanticscholar.org/6e86/1d0b58bdf7e2e2bb0ecbf274cee6974fe13f.pdf
    :param im:
    :param W: int width of the ridge
    :return: array
    """
    j1 = lambda x, y: 2 * x * y
    j2 = lambda x, y: x ** 2 - y ** 2
    j3 = lambda x, y: x ** 2 + y ** 2

    (y, x) = im.shape
    x_cuda, y_cuda = cuda.grid(2)
    # for gpu
    if x_cuda*W+1 < x and y_cuda*W+1 < y:

        x_cuda = x_cuda*W+1
        y_cuda = y_cuda*W+1

        nominator = 0
        denominator = 0
        for l in range(y_cuda, min(y_cuda + W, y - 1)):
            for k in range(x_cuda, min(x_cuda + W , x - 1)):
                Gx = round(Gx_[l, k])  # horizontal gradients at l, k
                Gy = round(Gy_[l, k])  # vertial gradients at l, k
                nominator += j1(Gx, Gy)
                denominator += j2(Gx, Gy)

                #nominator = round(np.sum(Gy_[j:min(j + W, y - 1), i:min(i + W , x - 1)]))
                #denominator = round(np.sum(Gx_[j:min(j + W, y - 1), i:min(i + W , x - 1)]))
        if nominator or denominator:
            angle = (math.pi + math.atan2(nominator, denominator)) / 2
            orientation = np.pi/2 + math.atan2(nominator,denominator)/2
            result[int((y_cuda-1) // W),int((x_cuda-1)//W) ] = angle
        else:
            result[int((y_cuda-1) // W),int((x_cuda-1)//W) ] = 0

            # segment image
            # focus_img = im[j:min(j + W, y - 1), i:min(i + W , x - 1)]
            # segmentator = -1 if segmentator/W*W < np.max(focus_img)*

        # for j in range(1, y, W):
        #     for i in range(1, x, W):
        #         nominator = 0
        #         denominator = 0
        #         for l in range(j, min(j + W, y - 1)):
        #             for k in range(i, min(i + W , x - 1)):
        #                 Gx = round(Gx_[l, k])  # horizontal gradients at l, k
        #                 Gy = round(Gy_[l, k])  # vertial gradients at l, k
        #                 nominator += j1(Gx, Gy)
        #                 denominator += j2(Gx, Gy)

        #         #nominator = round(np.sum(Gy_[j:min(j + W, y - 1), i:min(i + W , x - 1)]))
        #         #denominator = round(np.sum(Gx_[j:min(j + W, y - 1), i:min(i + W , x - 1)]))
        #         if nominator or denominator:
        #             angle = (math.pi + math.atan2(nominator, denominator)) / 2
        #             orientation = np.pi/2 + math.atan2(nominator,denominator)/2
        #             result[int((j-1) // W)].append(angle)
        #         else:
        #             result[int((j-1) // W)].append(0)

        #         # segment image
        #         # focus_img = im[j:min(j + W, y - 1), i:min(i + W , x - 1)]
        #         # segmentator = -1 if segmentator/W*W < np.max(focus_img)*


class OrientationNode(object):
    def __init__(self):
        self.bridge= CvBridge()
        self.rospack = rospkg.RosPack()
        self.rate = rospy.Rate(30)
        self.input_size = rospy.get_param('~input_size', 512)
        # self.signal_loop = rospy.get_param('~signal_loop', True)
        self.signal_loop = 0
        self.pub = rospy.Publisher("~output", Image, queue_size=1)
        self.pub_std = rospy.Publisher("~output/std", msg_std, queue_size=1)
        self.pub_debug = rospy.Publisher("~output/debug", Image, queue_size=1)
        self.pub_mask = rospy.Publisher("~output/mask", Image, queue_size=1)
        self.pub_crown_contour = rospy.Publisher("~output/crown_contour", Image, queue_size=1)
        self.pub_path = rospy.Publisher("~output/path", Int16MultiArray, queue_size=1)

        self.sub_signal_loop_once = None
        self.DIVANGLES = 18
        # self.sub_mask = rospy.Subscriber(
        #     "/hair_pcl_mask/output", Image, self.update_mask)

        # self.sub_image = rospy.Subscriber(
        #     "/hair_pcl_mask/output/masked", Image, self.process_image)

        # self.sub_mask = message_filters.Subscriber(
        #     "/hair_pcl_mask/output", Image)

        # self.sub_image =message_filters.Subscriber(
        #     "/hair_pcl_mask/output/masked", Image)
        
        # self.ts = message_filters.TimeSynchronizer([self.sub_image,self.sub_mask], 1000)
        # self.ts.registerCallback(self.process_image)

        self.sub_signal = rospy.Subscriber("/signal_loop", Int8, self.signal_cb)
        
        self.std_list_mean_before = 0
        self.smoth = True
        self.mask = None
        self.bbox_path = False
        self.multi_path = False
        self.crown_path = False
        self.linear_path = True
        self.sc = 1
        self.r = 32
        self.W = int(self.r*self.sc)
        self.gauss_k = 61
        
        self.ssigma = 4
        self.minVal = 127
        self.maxVal = 255

        self.max_iter_path = 30
        self.k = int(self.r*self.sc)
        self.p_0_y = int(self.r*self.sc)
        self.comb_offset = 1
        
        rospy.loginfo('hair orientation node started')

    # def make_grid(self,mask):

    # def update_mask(self,msg):
    #     self.mask= self.bridge.imgmsg_to_cv2(msg, "mono8")
    #     return
    def get_one_org_xyz(self, points,width,xy):
        start = width*xy[0]+ xy[1]
        point = points[start]
        return point

    def get_orgainized_xyz(self, points, height, width, point_step, row_step,fields, b, e):
        point_b =self.get_one_org_xyz(points,width,b)
        point_e = self.get_one_org_xyz(points,width,e)
        return (point_b, point_e)
    
    def point_cb(self, point_cloud, target):
        points = list(pc2.read_points(point_cloud, skip_nans=False))
        height = point_cloud.height
        width = point_cloud.width
        point_step = point_cloud.point_step
        row_step = point_cloud.row_step
        fields = point_cloud.fields
        # result = [self.get_orgainized_xyz(points, height, width, point_step, row_step, fields,b, e)
        #           for b, e in target]
        #rospy.loginfo(result)
        points_b=[]
        points_e=[]
        for b, e in target:
            points_b.append(self.get_one_org_xyz(points, width, b))
            points_e.append(self.get_one_org_xyz(points, width, e))
        points_b = np.array(points_b, dtype=np.float32)
        points_e = np.array(points_e, dtype=np.float32)
        np.save('points_b', points_b)
        return points_b, points_e

    def cal_rect_hair(self, cnt):
        x_, y_, w_, h_ = 0, 0, 0, 0

        # for cnt in cnts:
        x,y,w,h = cv2.boundingRect(cnt)
        if w_ + h_ < w + h :
            w_ = w
            h_ = h
        return w_,h_

    def find_contours(self, img):
        contours, hierarchy = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #hair_contours=[]
        if len(contours) != 0:
            hair_cnt =max(contours, key=cv2.contourArea)
        # for i, cnt in enumerate(contours):
        #     area = cv2.contourArea(cnt)
        #     if area > 8000:
        #         hair_contours.append(cnt)
            # print(f"contours[{i}].shape: {cnt.shape}")
        return hair_cnt

    def points_around_crown(self, crown):
        contours, _ = cv2.findContours(crown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=lambda x: cv2.contourArea(x))

        crown_ =cv2.cvtColor(crown, cv2.COLOR_GRAY2BGR)
        
        crown_contour = np.zeros_like(crown_)
        #print(np.array(contour).shape)
        
        cv2.drawContours(crown_contour, contour, -1, (0, 255, 0), thickness=2)
        return crown_contour

    def points_around_crown_lap(self, crown, p = 4):
        laplacian = cv2.Laplacian(crown, -1)
        lap_idx = np.argwhere(laplacian)
        y_to = int(lap_idx.shape[0]/2)
        s = 10
        x = range(0, crown.shape[1], s)
        y =[lap_idx[y_to,0]] * len(x)
        h_path=list(zip(y, x))
        
        lap_idx = lap_idx[y_to::p,:].tolist()
        
        lap_idx.extend(h_path)
        lap_idx = np.array(lap_idx)
        
        
        laplacian = cv2.cvtColor(laplacian, cv2.COLOR_GRAY2BGR)
        for xy in lap_idx:
            laplacian = cv2.circle(laplacian, (xy[1], xy[0]), 1, (0,255,0), 1) 

        
        return laplacian, lap_idx
    
    def detect_crown(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)

        # hsv_min = np.array([14, 196, 89])
        # hsv_max = np.array([60, 255, 147])
        hsv_min = np.array([15, 80, 80])
        hsv_max = np.array([60, 150, 150])

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
        
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)
        
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        return mask, masked_img

        
    def init_filters(self):
        filters = []
        ksize = 31
        for theta in np.arange(0, np.pi, np.pi / self.DIVANGLES):
            kern = cv2.getGaborKernel((ksize, ksize), 4.0, theta, 10.0, 0.5, 0, ktype=cv2.CV_32F)
            kern /= 1.5*kern.sum()
            filters.append(kern)
        return filters

    # フィルター処理
    def filterProcess(self, img, filters,SCALE=0.75):
        height, width = img.shape[:2]
        height = int(height * SCALE)
        width = int(width * SCALE)

        if len(img.shape) == 2:
            plane = 1               # 画像チャンネル数=1
            charcolor = (0,0,0)
            filterType = cv2.CV_8UC1
        else:
            plane = img.shape[2]    # 画像チャンネル数=3
            charcolor = (255)
            filterType = cv2.CV_8UC3
        tileimg = np.zeros((height*9, width*2, 3), np.uint8)    # タイリングイメージ

        accum = np.zeros_like(img)
        for kern, theta, i in zip(filters, np.arange(0, 180, 180 / self.DIVANGLES), range(0,self.DIVANGLES)):
            fimg = cv2.filter2D(img, filterType, kern)

            # 縮小、タイルイメージ
            x = (i % 2) * width
            y = (i // 2) * height
            rimg = cv2.resize(fimg, (width,height), interpolation=cv2.INTER_CUBIC)
            if plane == 1:
                tileimg[y:y+height, x:x+width] = cv2.merge((rimg,rimg,rimg))
            else:
                tileimg[y:y+height, x:x+width] = rimg

            # 最大取得
            np.maximum(accum, fimg, accum)
        return accum, tileimg

    # ガボールフィルタ処理
    def gaborFilter(self,img):
        filters = self.init_filters()
        res1,tileimg = self.filterProcess(img, filters)
        return res1

    def frequest(self,im, orientim, kernel_size, minWaveLength, maxWaveLength):
        """
        Based on https://pdfs.semanticscholar.org/ca0d/a7c552877e30e1c5d87dfcfb8b5972b0acd9.pdf pg.14
        Function to estimate the fingerprint ridge frequency within a small block
        of a fingerprint image.
        An image block the same size as im with all values set to the estimated ridge spatial frequency.  If a
        ridge frequency cannot be found, or cannot be found within the limits set by min and max Wavlength freqim is set to zeros.
        """
        rows, cols = np.shape(im)

        # Find mean orientation within the block. This is done by averaging the
        # sines and cosines of the doubled angles before reconstructing the angle again.
        cosorient = np.cos(2*orientim) # np.mean(np.cos(2*orientim))
        sinorient = np.sin(2*orientim) # np.mean(np.sin(2*orientim))
        block_orient = math.atan2(sinorient,cosorient)/2
        # Rotate the image block so that the ridges are vertical
        rotim = ndimage.rotate(im,block_orient/np.pi*180 + 90,axes=(1,0),reshape = False,order = 3,mode = 'nearest')

        # Now crop the image so that the rotated image does not contain any invalid regions.
        cropsze = int(np.fix(rows/np.sqrt(2)))
        offset = int(np.fix((rows-cropsze)/2))
        rotim = rotim[offset:offset+cropsze][:,offset:offset+cropsze]

        # Sum down the columns to get a projection of the grey values down the ridges.
        ridge_sum = np.sum(rotim, axis = 0)
        dilation = ndimage.grey_dilation(ridge_sum, kernel_size, structure=np.ones(kernel_size))
        ridge_noise = np.abs(dilation - ridge_sum); peak_thresh = 2;
        maxpts = (ridge_noise < peak_thresh) & (ridge_sum > np.mean(ridge_sum))
        maxind = np.where(maxpts)
        _, no_of_peaks = np.shape(maxind)

        # Determine the spatial frequency of the ridges by dividing the
        # distance between the 1st and last peaks by the (No of peaks-1). If no
        # peaks are detected, or the wavelength is outside the allowed bounds, the frequency image is set to 0
        if(no_of_peaks<2):
            freq_block = np.zeros(im.shape)
        else:
            waveLength = (maxind[0][-1] - maxind[0][0])/(no_of_peaks - 1)
            if waveLength>=minWaveLength and waveLength<=maxWaveLength:
                freq_block = 1/np.double(waveLength) * np.ones(im.shape)
            else:
                freq_block = np.zeros(im.shape)
        return(freq_block)


    def ridge_freq(self, im, mask, orient, block_size, kernel_size, minWaveLength, maxWaveLength):
        # Function to estimate the fingerprint ridge frequency across a
        # fingerprint image.
        rows,cols = im.shape
        freq = np.zeros((rows,cols))

        for row in range(0, rows - block_size, block_size):
            for col in range(0, cols - block_size, block_size):
                image_block = im[row:row + block_size][:, col:col + block_size]
                angle_block = orient[row // block_size][col // block_size]
                if angle_block:
                    freq[row:row + block_size][:, col:col + block_size] = self.frequest(image_block, angle_block, kernel_size, minWaveLength, maxWaveLength)

        freq = freq*mask
        freq_1d = np.reshape(freq,(1,rows*cols))
        ind = np.where(freq_1d>0)
        ind = np.array(ind)
        ind = ind[1,:]
        non_zero_elems_in_freq = freq_1d[0][ind]
        medianfreq = np.median(non_zero_elems_in_freq) * mask

        return medianfreq

    def gabor_filter(self,im, orient, freq, kx=0.65, ky=0.65):
        """
        Gabor filter is a linear filter used for edge detection. Gabor filter can be viewed as a sinusoidal plane of
        particular frequency and orientation, modulated by a Gaussian envelope.
        :param im:
        :param orient:
        :param freq:
        :param kx:
        :param ky:
        :return:
        """
        angleInc = 3
        im = np.double(im)
        rows, cols = im.shape
        return_img = np.zeros((rows,cols))

        # Round the array of frequencies to the nearest 0.01 to reduce the
        # number of distinct frequencies we have to deal with.
        freq_1d = freq.flatten()
        frequency_ind = np.array(np.where(freq_1d>0))
        non_zero_elems_in_freq = freq_1d[frequency_ind]
        non_zero_elems_in_freq = np.double(np.round((non_zero_elems_in_freq*100)))/100
        unfreq = np.unique(non_zero_elems_in_freq)

        # Generate filters corresponding to these distinct frequencies and
        # orientations in 'angleInc' increments.
        sigma_x = 1/unfreq*kx
        sigma_y = 1/unfreq*ky
        block_size = np.round(3*np.max([sigma_x,sigma_y]))
        array = np.linspace(-block_size,block_size,(2*block_size + 1))
        x, y = np.meshgrid(array, array)

        # gabor filter equation
        reffilter = np.exp(-(((np.power(x,2))/(sigma_x*sigma_x) + (np.power(y,2))/(sigma_y*sigma_y)))) * np.cos(2*np.pi*unfreq[0]*x)
        filt_rows, filt_cols = reffilter.shape
        gabor_filter = np.array(np.zeros((180//angleInc, filt_rows, filt_cols)))

        # Generate rotated versions of the filter.
        for degree in range(0,180//angleInc):
            rot_filt =ndimage.rotate(reffilter,-(degree*angleInc + 90),reshape = False)
            gabor_filter[degree] = rot_filt

        # Convert orientation matrix values from radians to an index value that corresponds to round(degrees/angleInc)
        maxorientindex = np.round(180/angleInc)
        orientindex = np.round(orient/np.pi*180/angleInc)
        for i in range(0,rows//16):
            for j in range(0,cols//16):
                if(orientindex[i][j] < 1):
                    orientindex[i][j] = orientindex[i][j] + maxorientindex
                if(orientindex[i][j] > maxorientindex):
                    orientindex[i][j] = orientindex[i][j] - maxorientindex

        # Find indices of matrix points greater than maxsze from the image boundary
        block_size = int(block_size)
        valid_row, valid_col = np.where(freq>0)
        finalind = \
            np.where((valid_row>block_size) & (valid_row<rows - block_size) & (valid_col>block_size) & (valid_col<cols - block_size))

        for k in range(0, np.shape(finalind)[1]):
            r = valid_row[finalind[0][k]]; c = valid_col[finalind[0][k]]
            img_block = im[r-block_size:r+block_size + 1][:,c-block_size:c+block_size + 1]
            return_img[r][c] = np.sum(img_block * gabor_filter[int(orientindex[r//16][c//16]) - 1])

        gabor_img = 255 - np.array((return_img < 0)*255).astype(np.uint8)

        return gabor_img

    def cal_pose(self, points:list):# points:pointcloud
        point_b = None
        for point in points:
            point_b = point
            point_a = point - point_b
        return path
    #cal xy to xy in angle map
    def cal_angle_map_xy(self, p_y, p_x, W):
        a_y = int(p_y // W)
        a_x = int(p_x // W)
        return a_y, a_x

    # calculate path
    def cal_path(self, p_0, mask, img, k, W, max_iter, angles):
        (y, x) = img.shape[:2]
        p_t = p_0
        path_ = []
        a_y, a_x = self.cal_angle_map_xy(p_t[0], p_t[1], W)
        angle_t = angles[a_y, a_x]
        # rospy.loginfo(p_t)
        # rospy.loginfo(angle_t)
        # rospy.loginfo(mask.shape)
        # rospy.loginfo(mask[p_t[0],p_t[1]])
        i = 0
        while (mask[p_t[0],p_t[1]] == 255 and p_t[0] < y-1 and p_t[1] < x-1 and i < max_iter):
            path_.append(p_t[::-1].copy())

            a_y, a_x = self.cal_angle_map_xy(p_t[0], p_t[1], W)
            # rospy.loginfo(a_y)
            # rospy.loginfo(a_x)
            if a_y == angles.shape[0]:
                a_y -= 1
            if a_x == angles.shape[1]:
                a_x -= 1

            angle_t = angles[a_y, a_x]
            # print(angle_t)

            p_t[0] = int(p_t[0] + k*np.sin(angle_t))
            p_t[1] = int(p_t[1] + k*np.cos(angle_t))

            # rospy.loginfo(np.sin(angle_t))
            if p_t[0]  < 0:
                p_t[0] = 0
            if p_t[1]  < 0:
                p_t[1] = 0
            i += 1
            #rospy.loginfo(path_)
        return path_

    # calculate path
    def cal_linear_path(self, p_0, mask, img, k, W, max_iter, angles):
        (y, x) = img.shape[:2]
        p_t = p_0
        path_ = []
        a_y, a_x = self.cal_angle_map_xy(p_t[0], p_t[1], W)
        a_y, a_x = self.cal_angle_map_xy(p_t[0], p_t[1], W)
        
        angle_t = angles[a_y, a_x]
        # rospy.loginfo(p_t)
        # rospy.loginfo(angle_t)
        # rospy.loginfo(mask.shape)
        # rospy.loginfo(mask[p_t[0],p_t[1]])
        i = 0
        while (mask[p_t[0],p_t[1]] == 255 and p_t[0] < y-1 and p_t[1] < x-1 and i < max_iter):
            path_.append(p_t[::-1].copy())

            a_y, a_x = self.cal_angle_map_xy(p_t[0], p_t[1], W)
            # rospy.loginfo(a_y)
            # rospy.loginfo(a_x)
            if a_y == angles.shape[0]:
                a_y -= 1
            if a_x == angles.shape[1]:
                a_x -= 1

            angle_t = np.pi/2
            # print(angle_t)

            p_t[0] = int(p_t[0] + k*np.sin(angle_t))
            p_t[1] = int(p_t[1] + k*np.cos(angle_t))

            # rospy.loginfo(np.sin(angle_t))
            if p_t[0]  < 0:
                p_t[0] = 0
            if p_t[1]  < 0:
                p_t[1] = 0
            i += 1
            #rospy.loginfo(path_)
        return path_

    # cal str
    def sliding_window(self, h, v, W,mask, angles):
        m, n = angles.shape[:2]
        max_mean = 0
        max_std = 0
        max_i, max_j= 0, 0

        std_list = []

        for i in range(m-h+1):
            for j in range(n-v+1):
                #rospy.loginfo(angles[i:i+h,j:j+v])
                if not (0 in mask[i*W:i*W+h*W,j*W:j*W+v*W]):
                    std = np.std(angles[i:i+h,j:j+v])
                    std_list.append(std)
                    if max_std < std:
                        max_std = std
                        max_i = i
                        max_j = j
        std_list_mean = np.mean(std_list)
        #print(std_list_mean)
        diff = np.abs(self.std_list_mean_before - std_list_mean)

        #print(diff)
        # if diff < 0.05:
        #     print("complete comb")
        #rospy.loginfo("max std: "+str(max_std)+" max_i: "+ str(max_i)+" max_j: " + str(max_j))

        self.std_list_mean_before = std_list_mean
        return max_std, max_i, max_j
    def sliding_window_weight(self, h, v, W,mask, angles):
        m, n = angles.shape[:2]
        max_mean = 0
        max_std = 0
        max_i, max_j= 0, 0
        std_list = []
        for i in range(m-h+1):
            for j in range(n-v+1):
                #rospy.loginfo(angles[i:i+h,j:j+v])
                if not (0 in mask[i*W:i*W+h*W,j*W:j*W+v*W]):
                    weight = 1-(i*W+h/2)/mask.shape[0]
                    if weight > 0.8:
                        weight = 1-weight
                    
                    height = mask.shape[0] - i*W+(h/2)
                    print(f"weight:{weight},height:{height}")
                    std = np.std(angles[i:i+h,j:j+v])*weight
                    std_list.append(std)
                    if max_std < std:
                        max_std = std
                        max_i = i
                        max_j = j
        std_list_mean = np.mean(std_list)
        #print(std_list_mean)
        diff = np.abs(self.std_list_mean_before - std_list_mean)

        #print(diff)
        # if diff < 0.05:
        #     print("complete comb")
        #rospy.loginfo("max std: "+str(max_std)+" max_i: "+ str(max_i)+" max_j: " + str(max_j))

        self.std_list_mean_before = std_list_mean
        return max_std, max_i, max_j

    def nothing(self):
        pass
    
    def img_hsv_viz(self, img):
        # ウィンドウの生成
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)

        # トラックバーの生成
        cv2.createTrackbar('minH', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('maxH', 'image', 255, 255, self.nothing)
        cv2.createTrackbar('minS', 'image', 0, 255, self.nothing )
        cv2.createTrackbar('maxS', 'image', 255, 255, self.nothing)
        cv2.createTrackbar('minV', 'image', 0, 255, self.nothing)
        cv2.createTrackbar('maxV', 'image', 255, 255, self.nothing)
  

        # HSV変換
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)

        while True:
            # トラックバーの値の取得
            minH = cv2.getTrackbarPos('minH', 'image')
            minS = cv2.getTrackbarPos('minS', 'image')
            minV = cv2.getTrackbarPos('minV', 'image')
            maxH = cv2.getTrackbarPos('maxH', 'image')
            maxS = cv2.getTrackbarPos('maxS', 'image')
            maxV = cv2.getTrackbarPos('maxV', 'image')

            # 画像の更新
            img_mask = cv2.inRange(img_hsv, np.array([minH, minS, minV]), np.array([maxH, maxS, maxV]))
            cv2.imshow('image', img_mask)

            # qキーで終了
            if cv2.waitKey(16) & 0xFF == ord('q'):
                break
    
    def signal_cb(self, msg_signal):
        # self.sub_signal_loop_once = rospy.Subscriber(
        #     "~input/signal_loop", Bool, self.update_signal_loop_once, self.sub_signal_loop_once)
        self.signal_loop = msg_signal.data

        start = time.time()
    
        msg_mask = rospy.wait_for_message("/hair_pcl_mask/output", Image)
        msg_image = rospy.wait_for_message("/hair_pcl_mask/output/masked", Image)
        
        try:
            self.mask = self.bridge.imgmsg_to_cv2(msg_mask, "mono8")
            self.mask = cv2.resize(self.mask, dsize=None, fx=self.sc, fy=self.sc)
        except Exception as err:
            rospy.logerr(err)
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg_image, "mono8")
            img = cv2.resize(img, dsize=None, fx=self.sc, fy=self.sc)

        except Exception as err:
            rospy.logerr(err)
            return

        # crown_msg = rospy.wait_for_message('/hair_pcl_mask/output/crown',Image)
        # crown = self.bridge.imgmsg_to_cv2(crown_msg, "mono8")
        # crown = cv2.resize(crown, dsize=None, fx=self.sc, fy=self.sc)

        # crown_contour_lap, crown_idx = self.points_around_crown_lap(crown)
        # print(crown_idx.shape)

    
        ret = len(img)
        if ret == 0:
            return

        #img_b = cv2.fastNlMeansDenoising(img_g,h=1)
        #self.img_hsv_viz(img_hsv)
        
        #mask_crown, img_crown = self.detect_crown(img)
        #mask_crown_inv = ~mask_crown
        #self.mask -= mask_crown

        # img_g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        img_h = clahe.apply(img)
        th3 = cv2.adaptiveThreshold(img_h,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
              cv2.THRESH_BINARY,11,2)
        # #_,bw = cv2.threshold(img_c, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        img_h = cv2.bitwise_not(th3)

        
        img_c = coh.coherence_filter(img_h, sigma=7, str_sigma=15, blend= 0.9, iter_n=5,gray_on=0)

        #img_c = self.gaborFilter(img_h)
        #print("mask")
        #img_c = img_h


        # Configure the blocks
        threadsperblock = (self.W, self.W)
        blockspergrid_x = int(math.ceil(img_c.shape[0] / threadsperblock[0]))
        blockspergrid_y = int(math.ceil(img_c.shape[1] / threadsperblock[1]))
        blockspergrid = (blockspergrid_x, blockspergrid_y)

        img_c_global_mem = cuda.to_device(img_c)

        angles = None
        sobelOperator = [[-1, 0, 1], [-2, 0, 1], [-1, 0, 1]]
        ySobel = np.array(sobelOperator).astype(np.int)
        xSobel = np.transpose(ySobel).astype(np.int)
        Gx_ = cv2.filter2D(img_c/125,-1, ySobel)*125
        Gy_ = cv2.filter2D(img_c/125,-1, xSobel)*125


        angles = np.array([[0 for j in range(1, img_c.shape[1], self.W)] for i in range(1, img_c.shape[0], self.W)]).astype(np.float)
        #print(angles.shape)

        angles_global_mem = cuda.to_device(angles)

        # angles = self.calculate_angles(img_c, angles, self.W, False)
        calculate_angles[blockspergrid, threadsperblock](img_c_global_mem, angles_global_mem, self.W, Gy_, Gx_)
        angles = angles_global_mem.copy_to_host()
        cnt = np.array([])
        if self.smoth:
            angles_smooth = self.smooth_angles(angles, self.gauss_k)
        try:
            cnt = self.find_contours(self.mask)
        except Exception as err:
            rospy.logerr(err)
            
        if cnt.any():
            w_,h_ = self.cal_rect_hair(cnt)
            w_ = w_/self.W
            h_ = h_/self.W
        else:
            return
            w_ = angles.shape[1]
            h_ = angles.shape[0]

        dv = 4
        dh = 4
        v = int(w_/dv)
        h = int(h_/dh)

        #v = 8 *self.W
        #h = int(img_l.shape[0]/d)
        #print(angles)

        #max_std, max_i, max_j = self.sliding_window_lap(h, v, img_l)
        # max_std, max_i, max_j = self.sliding_window(h, v, self.W, self.mask, angles)
        max_std, max_i, max_j = self.sliding_window_weight(h, v, self.W, self.mask, angles)

        # max_std, max_i, max_j = self.sliding_window(h, v, img_l)

        msg_std_ = msg_std()
        msg_std_.std = max_std
        msg_std_.i = max_i*2
        msg_std_.j = max_j*2

        # img_thc = cv2.cvtColor(img_th, cv2.COLOR_GRAY2BGR)
        # img_c = self.normalise(img_c)
        # mean_val = np.mean(img_c[self.mask==0])
        # std_val = np.std(img_c[self.mask==0])
        # norm_img = (img_c- mean_val)/(std_val)

        # freq = self.ridge_freq(img_c, self.mask, angles, self.W, kernel_size=5, minWaveLength=5, maxWaveLength=15)
        # print(freq[np.where(~np.isnan(freq))])
        #img_c = self.gaborFilter(img_c)

        line_coords, img_edge = self.visualize_angles(img_c, self.mask, angles, self.W)
        #rospy.loginfo(line_coords)
        #point_msg = rospy.wait_for_message('/realsense_torso/depth_registered/points', PointCloud2)
        #points_b, points_e = self.point_cb(point_msg, line_coords)

        #print(np.array(line_coords).shape)
        img_h = cv2.cvtColor(img_c, cv2.COLOR_GRAY2BGR)
        img_debug = np.copy(img_h)
        img_edge_ = np.copy(img_edge)

        y1, x1 = max_i*self.W, self.W*max_j
        y2, x2 = max_i*self.W+h*self.W, self.W*max_j+self.W*v

        d = 4
        y3, x3 = y1 + int((y2 -y1)/d) ,x2
        img_mask = np.full(img_h.shape[:2], 0)
        img_mask = img_mask.astype(np.uint8)

        img_mask[y1:y3, x1:x3] = 255
        img_debug[y1:y2, x1:x2] = (0,0, 127)

        img_debug[y1:y3, x1:x3] = (0,0, 127)

        img_edge_[y1:y2, x1:x2] = (0,0, 127)
        img_edge_[y1:y3, x1:x3] = (0,0, 127)

        dst = cv2.addWeighted(img_debug, 0.3, img_h, 0.7, 0)
        img_edge = cv2.addWeighted(img_edge_, 0.3, img_edge, 0.7, 0)

        comb_y = y1 - self.comb_offset
        comb_y = comb_y if comb_y >= 0 else 0
        comb_x = int((x1 + x3)/2)
        comb_point = [comb_y, comb_x]
        
        #cal_path
        paths = []
        if self.bbox_path:
            path = self.cal_path(comb_point, self.mask, img_c, self.k, self.W, self.max_iter_path ,angles_smooth)
            path = np.array(path, dtype=np.int16)
            paths.append(path)
        #multi demo path
        if self.multi_path:
            paths = []
            self.p_0_x = list(range(0, img_c.shape[1], self.W))
            self.p_0_xy = [[self.p_0_y, p] for p in self.p_0_x]
            for p_0 in self.p_0_xy:
                path = self.cal_path(p_0, self.mask, img_c, self.k, self.W, self.max_iter_path ,angles_smooth)
                path = np.array(path, dtype=np.int16)
                paths.append(path)
        
        #path from crown
        # if self.crown_path and crown_idx.any():
        #     for xy in crown_idx:
        #         path = self.cal_path(xy, self.mask, img_c, self.k, self.W, self.max_iter_path ,angles_smooth)
        #         paths.extend(path)

        #linear path
        if self.linear_path:
            path = self.cal_linear_path(comb_point, self.mask, img_c, self.k, self.W, self.max_iter_path, angles_smooth)
            paths.extend(path)
            
        #print(paths[0])
        # if path != None:
        #     print(path)
        #print(angles)
        
        # img_mask[y1:y2, x1:x2] = 255
        # img_ori[y1:y2, x1:x2] = (0,0, 127)

        # dst = cv2.addWeighted(img_debug, 0.5, img_ori, 0.5, 0)

        # img_ori = self.visualize_angles(img_l, self.mask, angles, self.W)
        #img_debug = np.copy(img_l)

        #img_l[max_i:max_i+h, max_j: int(max_j+v)] = 127
        #dst = cv2.addWeighted(img_debug, 0.5, img_l, 0.5, 0)
        #calculate mean and std of orientation map with grid
        #dst =  dst.astype(np.uint8)
        #publish output

        end = time.time()
        elapsed = end - start

        cv2.putText(img_edge, 'FPS:{:.2f}'.format(1/elapsed), (10, 30), cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 0), thickness=2)
        cv2.putText(dst, 'FPS:{:.2f}'.format(1/elapsed), (10, 30), cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 0), thickness=2)
        
        #visualize for path
        # for i,  path in enumerate(paths):
        #     i += 1
        for point in paths:
            cv2.circle(img_edge, (point[0], point[1]), 2, (0, 0, 255), -1)

        img_edge = img_edge.astype(np.uint8)
        #rospy.loginfo(path)
        #for resize path
        # if len(paths) > 0:
        #     paths = [(p*(1/self.sc)).astype(np.int16)[:,::-1] for p in paths if p.size > 0]
        #     print(len(paths))

        paths = np.array(paths)
        if paths.size > 0:
               paths = (paths*(1/self.sc)).astype(np.int16)[:,::-1]
        print(paths)
        # self.visualize_path(img_edge, path)
        
        msg_ori = self.bridge.cv2_to_imgmsg(dst, encoding="bgr8")
        img_edge = cv2.resize(img_edge, dsize=None, fx=1/self.sc, fy=1/self.sc)

        msg_debug = self.bridge.cv2_to_imgmsg(img_edge, encoding="bgr8")
        img_mask = cv2.resize(img_mask, dsize=None, fx=1/self.sc, fy=1/self.sc)

        msg_mask =  self.bridge.cv2_to_imgmsg(img_mask, encoding="mono8")

        # crown_contour = cv2.resize(crown_contour_lap, dsize=None, fx=1/self.sc, fy=1/self.sc)

        # msg_crown_contour =  self.bridge.cv2_to_imgmsg(crown_contour, encoding="bgr8")

        #demo
        msg_path = numpy2i16multi(paths)
        
        #rospy.loginfo("elapsed:{}s".format(elapsed))
        self.pub.publish(msg_ori)
        self.pub_debug.publish(msg_debug)
        self.pub_std.publish(msg_std_)
        self.pub_mask.publish(msg_mask)
        # self.pub_crown_contour.publish(msg_crown_contour)

        self.pub_path.publish(msg_path)

        # if self.signal_loop == 1:
        #     self.signal_loop = 0

    def laplace_of_gaussian(self, gray_img, sigma=1., kappa=0.75, pad=False):
        """
        Applies Laplacian of Gaussians to grayscale image.
        :param gray_img: image to apply LoG to
        :param sigma:    Gauss sigma of Gaussian applied to image, <= 0. for none
        :param kappa:    difference threshold as factor to mean of image values, <= 0 for none
        :param pad:      flag to pad output w/ zero border, keeping input image size
        """
        assert len(gray_img.shape) == 2
        img = cv2.GaussianBlur(gray_img, (0, 0), sigma) if 0. < sigma else gray_img
        img = cv2.Laplacian(img, cv2.CV_64F)
        rows, cols = img.shape[:2]
        # min/max of 3x3-neighbourhoods
        min_map = np.minimum.reduce(list(img[r:rows-2+r, c:cols-2+c]
                                     for r in range(3) for c in range(3)))
        max_map = np.maximum.reduce(list(img[r:rows-2+r, c:cols-2+c]
                                     for r in range(3) for c in range(3)))
        # bool matrix for image value positiv (w/out border pixels)
        pos_img = 0 < img[1:rows-1, 1:cols-1]
        # bool matrix for min < 0 and 0 < image pixel
        neg_min = min_map < 0
        neg_min[1 - pos_img] = 0
        # bool matrix for 0 < max and image pixel < 0
        pos_max = 0 < max_map
        pos_max[pos_img] = 0
        # sign change at pixel?
        zero_cross = neg_min + pos_max
        # values: max - min, scaled to 0--255; set to 0 for no sign change
        value_scale = 255. / max(1., img.max() - img.min())
        values = value_scale * (max_map - min_map)
        values[1 - zero_cross] = 0.
        # optional thresholding
        if 0. <= kappa:
            thresh = float(np.absolute(img).mean()) * kappa
            values[values < thresh] = 0.
            log_img = values.astype(np.uint8)
        if pad:
            log_img = np.pad(log_img, pad_width=1, mode='constant', constant_values=0)
        return log_img



    def gauss(self, x, y):
        return (1 / (2 * math.pi * self.ssigma)) * math.exp(-(x * x + y * y) / (2 * self.ssigma))


    def kernel_from_function(self, size, f):
        kernel = [[] for i in range(0, size)]
        for i in range(0, size):
            for j in range(0, size):
                kernel[i].append(f(i - size / 2, j - size / 2))
        return kernel


    def smooth_angles(self, angles, k):
        """
        reference: https://airccj.org/CSCP/vol7/csit76809.pdf pg91
        Practically, it is possible to have a block so noisy that the directional estimate is completely false.
        This then causes a very large angular variation between two adjacent blocks. However, a
        fingerprint has some directional continuity, such a variation between two adjacent blocks is then
        representative of a bad estimate. To eliminate such discontinuities, a low-pass filter is applied to
        the directional board.
        :param angles:
        :return:
        """
        angles = np.array(angles)
        cos_angles = np.cos(angles.copy())#removed *2
        sin_angles = np.sin(angles.copy())#removed *2

        kernel = np.array(self.kernel_from_function(k, self.gauss))

        cos_angles = cv2.filter2D(cos_angles/125,-1, kernel)*125
        sin_angles = cv2.filter2D(sin_angles/125,-1, kernel)*125
        smooth_angles = np.arctan2(sin_angles, cos_angles)
                
        return smooth_angles
    def normalise(self,img):
        return (img - np.mean(img))/(np.std(img))

    def get_line_ends(self, i, j, W, tang):
        if -1 <= tang and tang <= 1:
            begin = (i, int((-W/2) * tang + j + W/2))
            end = (i + W, int((W/2) * tang + j + W/2))
        else:
            begin = (int(i + W/2 + W/(2 * tang)), j + W//2)
            end = (int(i + W/2 - W/(2 * tang)), j - W//2)
        return (begin, end)

    def visualize_angles(self, im, mask, angles, W):
        (y, x) = im.shape
        xy_list = []
        result = cv2.cvtColor(np.zeros(im.shape, np.uint8), cv2.COLOR_GRAY2RGB)
        mask_threshold = (W-1)**2
        for i in range(1, x, W):
            for j in range(1, y, W):
                radian = np.sum(mask[j - 1:j + W, i-1:i+W])
                if radian > mask_threshold:
                    tang = math.tan(angles[(j - 1) // W][(i - 1) // W])
                    (begin, end) = self.get_line_ends(i, j, W, tang)
                    xy_list.append((begin, end))
                    cv2.line(result, begin, end, color=(255,255,255))
        cv2.resize(result, im.shape, result)
        return xy_list, result

if __name__=="__main__":
    rospy.init_node("hair_orientation")
    app = OrientationNode()
    rospy.spin()

# red_point_detection.py
#!/usr/bin/env python3
import time, os, sys, signal, threading

import numpy as np 
import rospy
from sensor_msgs.msg import PointCloud2

class RedPointsDetectionNode(object): 
    def __init__(self):
        super(RedPointsDetectionNode, self).__init__()
        self.data = PointCloud2()
        self.point_step = 32
        self.R_COL = 18
        self.G_COL = 17
        self.B_COL = 16
        rospy.init_node("red_points_detection_node")
        rospy.loginfo("red points detection start")
 
    # Subscriber         
    def sub_pointcloud(self): 
        def callback(data): 
            self.sub_data = data
        sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback = callback) 
        rospy.spin() 
 
    # Publisher 
    def pub_red_pointcloud(self): 
        pub = rospy.Publisher("red_point_cloud", PointCloud2, queue_size = 1) 
        while not rospy.is_shutdown():
            try:
                pub_data = self.red_point_detection(self.sub_data)
                pub.publish(pub_data)
            except Exception as e:
                print(e)
                pass
    def red_point_detection(self, sub_data):
        red_point_data = sub_data
        red_pointcloud = np.array([d for d in sub_data.data]).reshape(-1, self.point_step)
        r_flags = red_pointcloud < 180
        g_flags = red_pointcloud > 130
        b_flags = red_pointcloud > 130
        R_under_threshold_row = np.where(r_flags)[0][np.where(np.where(r_flags)[1]==self.R_COL)]
        G_over_threshold_row = np.where(g_flags)[0][np.where(np.where(g_flags)[1]==self.G_COL)]
        B_over_threshold_row = np.where(b_flags)[0][np.where(np.where(b_flags)[1]==self.B_COL)]
        not_red_row = np.unique(np.concatenate([R_under_threshold_row, G_over_threshold_row, B_over_threshold_row]))

        red_pointcloud = np.delete(red_pointcloud, not_red_row, axis=0).ravel().tolist()
        red_point_data.width = int(len(red_pointcloud) / self.point_step)
        red_point_data.height = 1
        red_point_data.row_step = sub_data.width * self.point_step
        red_point_data.data = red_pointcloud
        rospy.loginfo("red pointcloud {}".format(int(len(red_pointcloud) / self.point_step)))
        return red_point_data

    # node start to subscribe and publish 
    def start_node(self): 
        sub_pointcloud = threading.Thread(target = self.sub_pointcloud) 
        pub_red_pointcloud = threading.Thread(target = self.pub_red_pointcloud) 
 
        sub_pointcloud.setDaemon(True) 
        sub_pointcloud.start() 
        pub_red_pointcloud.setDaemon(True) 
        pub_red_pointcloud.start() 
 
        sub_pointcloud.join() 
        pub_red_pointcloud.join() 
 
if __name__ == "__main__": 
    Red_points_detector = RedPointsDetectionNode() 
    Red_points_detector.start_node() 
    pass 

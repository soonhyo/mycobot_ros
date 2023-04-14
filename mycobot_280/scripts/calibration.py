# mycobot_position_calibration.py
#!/usr/bin/env python3
import time, os, sys, signal, threading

import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2

class MyCobotPositionCalibrationNode(object):
    def __init__(self):
        super(MyCobotPositionCalibrationNode, self).__init__()
        rospy.init_node("mycobot_position_calibration_node")
        rospy.loginfo("mycobot position calibration start")

    # Subscriber
    def sub_pointcloud(self):
        def callback(data):
            self.data = data
            rospy.loginfo("subscribed pointcloud")
            xyz_generator = pc2.read_points(self.data, field_names = ("x", "y", "z"), skip_nans=True)
            xyz_list = [xyz for xyz in xyz_generator if (abs(xyz[0]) < 0.8 and abs(xyz[1]) < 0.8 and abs(xyz[2]) < 0.8)]
            xyz_array = np.array(xyz_list)

            if len(xyz_list) > 3:
                marker_centroids = self.kmeans(3, xyz_array)
                rospy.loginfo("\n marker positions\n{}".format(marker_centroids))
                translation = self.cal_translation(marker_centroids)
                rospy.loginfo("\n translation\n{}".format(translation))

        sub = rospy.Subscriber("/hsi_color_filter/hsi_output", PointCloud2, callback = callback)
        rospy.spin()

    # node start
    def start_node(self):
        sub_pointcloud = threading.Thread(target = self.sub_pointcloud)
        sub_pointcloud.setDaemon(True)
        sub_pointcloud.start()
        sub_pointcloud.join()

    # clustering method
    def kmeans(self, k, X, max_iter=30): # k: cluster num, X: numpy array
        data_size, n_features = X.shape
        centroids = X[np.random.choice(data_size, k)]
        new_centroids = np.zeros((k, n_features))
        cluster = np.zeros(data_size)
        for epoch in range(max_iter):
            for i in range(data_size):
                distances = np.sum((centroids - X[i]) ** 2, axis=1)
                cluster[i] = np.argsort(distances)[0]
            for j in range(k):
                new_centroids[j] = X[cluster==j].mean(axis=0)
            if np.sum(new_centroids == centroids) == k:
                break
            centroids = new_centroids
        max_norm = 0
        min_norm = 0
        sorted_centroids = []
        for centroid in centroids:
            norm = centroid[2]
            if norm > max_norm:
                sorted_centroids.append(centroid)
                max_norm = norm
                if min_norm == 0:
                    min_norm = sorted_centroids[0][2]
            else:
                if norm > min_norm and min_norm != 0:
                    sorted_centroids.insert(1, centroid)
                else:
                    sorted_centroids.insert(0, centroid)
                    min_norm = norm
        sorted_centroids = np.array(sorted_centroids)

        return sorted_centroids

    # translation angles calculation
    ## calculation
    def cal_translation(self, marker_points):
        # マーカー1, 2, 3の位置ベクトル
        a_1, a_2, a_3 = marker_points
        # カメラからロボットへのベクトル
        V_robot = self.cal_robot_position_vector(a_2, a_3)
        # ロボットのXYZ単位ベクトル
        V_X = (a_2 - V_robot) / (np.linalg.norm(a_2 - V_robot))
        V_Y = (a_1 - V_robot) / (np.linalg.norm(a_1 - V_robot))
        V_Z = self.cal_normal_vector(marker_points)
        # カメラの水平面に対する仰角
        theta_1 = - (np.pi/2 - self.cal_subtended_angle(-V_robot, V_Z))
        # カメラの正面方向の回転角
        V_Y_camera = np.array([0, 1, 0])
        V_Y_camera_rotated = self.cal_rotate_vector_xaxis(V_Y_camera, -theta_1)
        theta_2 = - self.cal_subtended_angle(V_Z, -V_Y_camera_rotated)
        # カメラとロボットのそれぞれの正面方向とのなす角
        _, V_robot_projected_to_plane = self.cal_vector_projection(V_robot, V_Z)
        theta_3 = self.cal_subtended_angle(V_Y, V_robot_projected_to_plane)
        # mycobotの位置を土台の高さ0.027 m, V_Z方向に平行移動
        V_robot = V_robot + 0.027*V_Z

        return V_robot, theta_1, theta_2, theta_3


    ## vector and angle caluculation
    def cal_robot_position_vector(self, a_2, a_3):
        return (a_2 + a_3) / 2

    def cal_normal_vector(self, marker_points):
        a_1 = marker_points[0]
        a_2 = marker_points[1]
        a_3 = marker_points[2]
        A_12 = a_2 - a_1
        A_13 = a_3 - a_1
        cross = np.cross(A_13, A_12)
        return cross / np.linalg.norm(cross)

    def cal_subtended_angle(self, vec_1, vec_2):
        dot = np.dot(vec_1, vec_2)
        norm_1 = np.linalg.norm(vec_1)
        norm_2 = np.linalg.norm(vec_2)
        return np.arccos( dot / (norm_1 * norm_2) )

    def cal_vector_projection(self, org_vec, normal_vec):
        # org_vec: 射影したいベクトル
        # normal_vec: 射影したい平面の法線ベクトル
        projected_to_vertical = np.dot(org_vec, normal_vec) * normal_vec
        projected_to_horizontal = org_vec + projected_to_vertical
        return projected_to_vertical, projected_to_horizontal

    def cal_rotate_vector_xaxis(self, vec, angle):
        rotate_mat = np.array([[1, 0, 0], [0, np.cos(angle), np.sin(angle)], [0, -np.sin(angle), np.cos(angle)]])
        return vec.dot(rotate_mat)

    def cal_rotate_vector_yaxis(self, vec, angle):
        rotate_mat = np.array([[np.cos(angle), 0, -np.sin(angle)], [0, 1, 0], [np.sin(angle), 0, np.cos(angle)]])
        return vec.dot(rotate_mat)

    def cal_rotate_vector_zaxis(self, vec, angle):
        rotate_mat = np.array([[np.cos(angle), np.sin(angle), 0], [-np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        return vec.dot(rotate_mat)

if __name__ == "__main__":
    mycobot_position_calibrator = MyCobotPositionCalibrationNode()
    mycobot_position_calibrator.start_node()
    pass

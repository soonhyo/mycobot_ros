#!/usr/bin/env python
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import pyransac3d as pyrsc


class PointCloudListener:
    def __init__(self):
        # self.circle = pyrsc.Circle()
        self.point = pyrsc.Point()

        rospy.init_node('pointcloud2_listener', anonymous=True)
        rospy.Subscriber('/hsi_color_filter/hsi_output', PointCloud2, self.callback)
        rospy.spin()

    def dbscan(self, pcd):
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(pcd.cluster_dbscan(eps=0.1, min_points=5, print_progress=True))
        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        # Compute cluster centers
        centers = []
        for i in range(max(labels) + 1):
            cluster_indices = np.where(labels == i)[0]
            cluster_pcd = pcd.select_by_index(cluster_indices)
            cluster_center = cluster_pcd.get_center()
            centers.append(cluster_center)
        # Assume camera position is at [0, 0, 0]
        camera_position = np.array([0, 0, 0])
        # Compute distance from camera to each cluster center
        distances = [np.linalg.norm(camera_position - center) for center in centers]

        # Find index of the closest cluster
        closest_cluster_idx = np.argmin(distances)

        # Select the closest cluster
        closest_cluster_indices = np.where(labels == closest_cluster_idx)[0]
        closest_cluster_pcd = pcd.select_by_index(closest_cluster_indices)

        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0 #noise
        # without_noise = np.where(~(labels < 0))
        # ind = without_noise[np.argmax(np.array([len(ind) for ind in without_noise ]))]
        # # print(ind)
        # pcd = pcd.select_by_index(ind)
        # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        return closest_cluster_pcd

    def callback(self, data):
        try:
            points = list(point_cloud2.read_points(data, skip_nans=True))
            points = np.asarray(points, dtype=np.float32)[:, :3]

            # center, axis, radius, inliers = self.circle.fit(points, thresh=0.2, maxIteration=100)
            # center, inliers = self.point.fit(points, thresh=0.2, maxIteration=100)

            # create Open3D point cloud object
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.paint_uniform_color([0, 1, 0])

            #DBSCAN for clustering
            try:
                pcd = self.dbscan(pcd)
            except:
                return

            center = np.asarray(pcd.get_center())

            pcd_center = o3d.geometry.PointCloud()
            pcd_center.points = o3d.utility.Vector3dVector(np.array([center]))
            pcd_center.paint_uniform_color([1, 0, 0])

            print(center)

            # visualize point cloud
            o3d.visualization.draw_geometries([pcd, pcd_center])
        except Exception as e:
            rospy.logerr("Error processing point cloud data: {}".format(str(e)))

if __name__ == '__main__':
    pcl = PointCloudListener()

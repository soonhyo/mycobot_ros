import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import tf
from pynput import keyboard
import sys

class PointCloudSubscriber:
    def __init__(self):
        self.num_points = 0
        self.frame_id = ""
        rospy.Subscriber("/hsi_color_filter/hsi_output", PointCloud2, self.callback)
        self.listener = tf.TransformListener()
        self.listener_cam = tf.TransformListener()

        self.camera_points = []
        self.target_points = []
        # Wait for the transformation between the robot's end effector and the base frame
        self.listener.waitForTransform('/link1', '/link7', rospy.Time(), rospy.Duration(0.1))
        self.listener_cam.waitForTransform('/camera_link', '/camera_color_optical_frame', rospy.Time(), rospy.Duration(0.1))
        self.center = None
        self.link_point = None

        with keyboard.GlobalHotKeys({'q': self.my_function, 's': self.save_point}) as h:
            h.join()
    def dbscan(self, pcd):
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10, print_progress=False))
        max_label = labels.max()
        # print(f"point cloud has {max_label + 1} clusters")
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
        return closest_cluster_pcd

    def my_function(self):
        print("Function executed!")
        sys.exit()

    def save_point(self):
        if self.center.any() and len(self.camera_points) < 4:
            # print(self.center)
            self.target_points.append(self.link_point)
            self.camera_points.append(self.center)
            print("point is saved")

            if len(self.camera_points) == 4:
                print("4 points is saved, please continue to calculate the transform")
                self.calculate_transform(self.camera_points, self.target_points)
                self.camera_points = []
                self.target_points = []

    def rotation_matrix_to_rpy(self,R):
        sy = np.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    def calculate_transform(self, camera, link):
        A = np.array(link).T
        B = np.array(camera).T
        A = np.vstack((A, np.ones((1,4))))
        B = np.vstack((B, np.ones((1,4))))

        # Calculate the transformation matrix using the method of least squares
        # transform_matrix, residuals, rank, singular_values = np.linalg.lstsq(source_np_homogeneous, target_np_homogeneous, rcond=None
        T = np.dot(A, np.linalg.inv(B))

        # Extract translation term
        translation = T[:3, 3]

        # Extract rotation term
        rotation = T[:3, :3]
        rpy = self.rotation_matrix_to_rpy(rotation)
        # Print the transformation matrix
        print("rpy:\n",*rpy)
        print("trans:\n",*translation)
        # print(T)
        return T

    def callback(self, msg):
        # self.num_points = len(data.data) / data.point_step
        self.frame_id = msg.header.frame_id
        points = list(pc2.read_points(msg, skip_nans=True))

        try:
            # Get the transformation from the end effector frame to the base frame
            (trans, rot) = self.listener.lookupTransform('/link1', '/link7', rospy.Time(0))
            (trans_cam, rot_cam) = self.listener_cam.lookupTransform('/camera_link', '/camera_color_optical_frame', rospy.Time(0))

            self.link_point = trans
            # Print the translation vector (i.e. the coordinates of the end effector in the base frame)
            # print(f"End effector coordinates: x = {trans[0]}, y = {trans[1]}, z = {trans[2]}")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Convert numpy array to Open3D point cloud

        try:
            pcd = o3d.geometry.PointCloud()
            points = np.asarray(points, dtype=np.float32)[:,:3]
            R =tf.transformations.quaternion_matrix(rot_cam)
            TL =np.array(trans_cam).reshape(3,1)
            TL = np.vstack((TL, np.ones((1,1))))
            T_cam = np.hstack((R[:,:3], TL))

            pcd.points = o3d.utility.Vector3dVector(points)
            self.find_pointcloud_center(pcd, T_cam)
            # print("Message frame ID: %s" % self.frame_id)

        except Exception as e:
            return

    def find_pointcloud_center(self, pcd, T):
        # Perform KNN clustering to find the center
        # kdtree = o3d.geometry.KDTreeFlann(pcd)
        # print(np.asarray(pcd.points))
        # print(np.mean(pcd.points, axis=0))
        # [k, idx, _] = kdtree.search_knn_vector_3d(np.mean(pcd.points, axis=0), 1)
        # center = points[idx[0][0]]
        # print(idx)
        # center = np.asarray(pcd.points)[idx[0],:]
        # center =np.mean(pcd.points, axis=0)
        #DBSCAN for clustering
        try:
            pcd = self.dbscan(pcd)
        except:
            return

        center = np.asarray(pcd.get_center())

        # pcd_center = o3d.geometry.PointCloud()
        # pcd_center.points = o3d.utility.Vector3dVector(np.array([center]))
        # pcd_center.paint_uniform_color([1, 0, 0])

        self.center = np.dot(T, np.append(center,1).T)[:3]

if __name__=="__main__":
    rospy.init_node('pointcloud_subscriber')
    app = PointCloudSubscriber()
    rospy.spin()

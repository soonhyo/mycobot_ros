#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy

#import pyransac3d as pyrsc
import open3d as o3d

import rospy

from converter import i16multi2numpy

import quaternion

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pc2

#np.set_printoptions(threshold=sys.maxsize)

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

class TrajectoryHair(object):
    def __init__(self):
        rospy.Subscriber("/hair_orientation/output/path", Int16MultiArray, self.path_cb)
        #self.pub_path =rospy.Publisher("~output/path", PointCloud2, queue_size=1)
        self.pub_pose_array =rospy.Publisher("~output/pose_array", PoseArray, queue_size=1)
        self.bridge = CvBridge()
        self.all_viz = True

    def ransac_sphere(self, pcd):
        sph = pyrsc.Sphere()
        center, radius, inliers = sph.fit(np.asarray(pcd.points), thresh=0.01)
        return center, radius, inliers
    def ransac_plane(self, pcd, viz):
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                         ransac_n=3,
                                                         num_iterations=1000)
        inline = pcd.select_by_index(inliers).paint_uniform_color([1, 0, 0])
        outline = pcd.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])

        if viz and self.all_viz:
            o3d.visualization.draw_geometries([inline, outline],"ransac_plane")


        return plane_model, inline, outline
    def norm(self,x):
        return x/np.linalg.norm(x)

    def get_axis_y(self,pt_b,pt_a,n):

        v = pt_b - pt_a

        #unit_v = self.norm(v)

        axis_y = v - n.dot(v)
        return self.norm(axis_y)

    def get_axis_z(self, pcd, ind):
        return self.norm(np.asarray(pcd.normals)[ind[1:],:])

    def get_axis_x(self, y, z):
        return self.norm(np.cross(y,z))

    def get_axis(self, pcd, pcd_path):
        result_z =[]
        result_y =[]
        result_x =[]

        for pt_b,pt, pt_a in zip(np.asarray(pcd_path.points)[:-2],np.asarray(pcd_path.points)[1:-1], np.asarray(pcd_path.points)[2:]):
            downpcd, points_nst, idx_nst = self.search_neighboring_points(pcd, pt, 2)
            #print(points_nst.shape)
            pcd_nst = o3d.geometry.PointCloud()
            pcd_nst.points = o3d.utility.Vector3dVector(points_nst)

            axis_z = self.get_axis_z(downpcd, idx_nst)[0]
            result_z.append(axis_z)
            axis_y = self.get_axis_y(pt_b , pt_a, axis_z)
            result_y.append(axis_y)
            axis_x = self.get_axis_x(axis_y, axis_z)
            result_x.append(axis_x)
        return result_x, result_y,result_z

    def dbscan(self, pcd):
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(pcd.cluster_dbscan(eps=0.015, min_points=35, print_progress=True))
        max_label = labels.max()
        #print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0 #noise
        without_noise = np.where(~(labels < 0))
        print(np.argmax(np.array([len(ind) for ind in without_noise ])))
        ind = without_noise[np.argmax(np.array([len(ind) for ind in without_noise ]))]

        print(ind)
        pcd = pcd.select_by_index(ind)
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        return pcd

    def get_bbox(self, pcd, viz):
        aabb = pcd.get_axis_aligned_bounding_box()
        aabb.color = (1, 0, 0)
        obb = pcd.get_oriented_bounding_box()
        obb.color = (0, 1, 0)
        if viz and self.all_viz:
            o3d.visualization.draw_geometries([pcd],"bbox")

        return aabb, obb

    def get_normal(self, pcd, r, n, viz):
        print("Recompute the normal of the downsampled point cloud")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=n))

        #pcd.orient_normals_consistent_tangent_plane(100)
        #pcd.orient_normals_towards_camera_location(camera_location)
        pcd.orient_normals_to_align_with_direction()
        if viz and self.all_viz:
            o3d.visualization.draw_geometries([pcd],"normal",point_show_normal=True)

        return pcd

    def search_neighboring_points(self, pcd, target_pnts, num):
        pcd.paint_uniform_color([0.5, 0.5, 0.5])
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        #print("Find its nearest neighbors, and paint them blue.")#knn or radius
        [k, idx, _] = pcd_tree.search_knn_vector_3d(target_pnts, num)
        np.asarray(pcd.colors)[idx[1:],:] = [1, 0, 0]
        points_nst = np.asarray(pcd.points)[idx[1:],:]
        return pcd, points_nst, idx

    def search_neighboring_points_radius(self, pcd, target_pnts, r):
        pcd.paint_uniform_color([0.5, 0.5, 0.5])
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        #print("Find its nearest neighbors, and paint them green.")#knn or radius
        [k, idx, _] = pcd_tree.search_radius_vector_3d(target_pnts, r)
        np.asarray(pcd.colors)[idx[1:],:] = [0, 1, 0]
        points_nst = np.asarray(pcd.points)[idx[1:],:]
        return pcd, points_nst, idx

    def search_neighboring_points_rknn(self, pcd, target_pnts, r, n):
        pcd.paint_uniform_color([0.5, 0.5, 0.5])
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        #print("Find its nearest neighbors, and paint them green.")#knn or radius
        [k, idx, _] = pcd_tree.search_hybrid_vector_3d(target_pnts, r, n)
        np.asarray(pcd.colors)[idx[1:],:] = [0, 1, 0]
        points_nst = np.asarray(pcd.points)[idx[1:],:]
        return pcd, points_nst, idx

    def get_one_org_xyz(self, points,width, xy):
        ind = width*xy[0]+ xy[1]
        point = points[ind]
        return point, ind

    def point_cb(self, points,width, target):
        result=[]
        for p in target:
            point, ind=self.get_one_org_xyz(points, width, p)
            result.append(point)
        result = np.asarray(result, dtype=np.float32)[:,:3]
        result = result[~np.isnan(result).any(axis=1)]
        return result

    def visualize_tf(self, pcd, pcd_path, R, viz=False):
        meshs = []
        for pp,r in zip(pcd_path.points, R):
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            mesh.scale(0.01, center=(0,0,0))
            mesh.rotate(r, center=(0,0,0))
            mesh.translate(pp)
            meshs.append(mesh)
        if viz and self.all_viz:
            o3d.visualization.draw_geometries([pcd, *meshs],"transformation")

    def get_pose(self, pos, qtn):
         pose = Pose()
         pose.position.x = pos[0]
         pose.position.y = pos[1]
         pose.position.z = pos[2]
         pose.orientation.x = qtn.x
         pose.orientation.y = qtn.y
         pose.orientation.z = qtn.z
         pose.orientation.w = qtn.w
         return pose

    def get_pose_array(self, positions, qtns):
        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_depth_optical_frame"
        for position, qtn in zip(positions, qtns):
            pose_array.poses.append(self.get_pose(position, qtn))
        return pose_array

    def path_cb(self, msg_path):
        path = i16multi2numpy(msg_path)
        points_msg = rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)

        mask_msg = rospy.wait_for_message('/hair_pcl_mask/output',Image)
        mask = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")

        crown_msg = rospy.wait_for_message('/hair_pcl_mask/output/crown',Image)
        crown = self.bridge.imgmsg_to_cv2(crown_msg, "mono8")

        rospy.loginfo(mask.shape)

        ### point cloud processing ###

        #origin points
        points_ori = list(pc2.read_points(points_msg, skip_nans=False))
        width = points_msg.width
        
        # mask points
        mask_idx = np.argwhere(mask>0)
        if crown.any():
            crown_idx = np.argwhere(crown>0)

        # ori_idx = np.argwhere(mask!=None)
        # print("mask_idx:", mask_idx)
        # print("path_idx:", path)
        
        points_path = self.point_cb(points_ori, width, path)
        points_mask = self.point_cb(points_ori, width, mask_idx)
        if crown.any():
            points_crown = self.point_cb(points_ori, width, crown_idx)

        # points_ = self.point_cb(points_ori, width, ori_idx)

        # points_mask_ = deepcopy(points_mask)
        # find path points

        #print("points_path:", points_path)
        # print("points_mask:", points_mask)
        ### visualization for debug ###
        pcd_mask = o3d.geometry.PointCloud()
        pcd_mask.points = o3d.utility.Vector3dVector(points_mask )
        pcd_mask.paint_uniform_color([0.5, 0.5, 0.5])

        if crown.any():
            pcd_crown = o3d.geometry.PointCloud()
            pcd_crown.points = o3d.utility.Vector3dVector(points_crown)
            pcd_crown.paint_uniform_color([0, 0, 1])
        # o3d.visualization.draw_geometries([pcd_mask, pcd_crown],"mask and crown")
        
        # pcd_mask.paint_uniform_color([1, 0, 0])

        # points_ = np.array(points_)[:,:3]
        # pcd_ori = o3d.geometry.PointCloud()
        # pcd_ori.points = o3d.utility.Vector3dVector(points_)

        pcd_path = o3d.geometry.PointCloud()
        pcd_path.points = o3d.utility.Vector3dVector(points_path)
        pcd_path.paint_uniform_color([1, 0, 0])
        #o3d.visualization.draw_geometries([pcd_mask,pcd_path])

        #downsampling
        print("Downsample the point cloud with a voxel of 0.05")
        downpcd = pcd_mask.voxel_down_sample(voxel_size=0.005)

        #DBSCAN for clustering
        downpcd = self.dbscan(downpcd)

        #normal vector
        downpcd = self.get_normal(downpcd, r= 5, n= 200, viz = False)

        #rnn
        # downpcd, points_nst, idx = self.search_neighboring_points_radius(downpcd, pcd_path.points[-1], 0.01)

        # pcd_rnn = o3d.geometry.PointCloud()
        # pcd_rnn.points = o3d.utility.Vector3dVector(points_nst)
        # pcd_rnn.paint_uniform_color([0, 1, 0])

        # rnn_plane_model, rnn_inliers = self.ransac_plane(pcd_rnn)
        # rnn_az = self.norm(rnn_plane_model[:3])
        # print(rnn_az)
        # np.asarray(downpcd.normals)[idx[1:],:] = rnn_az

        # if True:
        #     o3d.visualization.draw_geometries([downpcd, pcd_rnn],"rnn", point_show_normal=True)

        ax, ay, az =self.get_axis(downpcd, pcd_path)

        # change coords for baxter with comb system : z>x x>z y>-y
        # tmp = az
        # az = ax
        # ax = tmp
        # ay = np.asarray(ay)*-1
        # tmp = ax
        # ax = ay
        # ay = np.asarray(tmp)*-1
        #todo low pass filter for ax ay az
        #R = np.array(list(zip(ax, ay, az))).transpose(2,1,0)
        R = np.array(list(zip(ax, ay, az)))
        
        q = quaternion.from_rotation_matrix(R)
        position = np.array(pcd_path.points)
        print(position)
        pose_array = self.get_pose_array(position, q)

        #print("R:",R)
        #print("q:",q[0].x)

        self.pub_pose_array.publish(pose_array)

        #visuzlization for path's tf
        self.visualize_tf(downpcd, pcd_path, R, viz=True)

        #get bounding box
        #aabb, obb = self.get_bbox(downpcd, viz=False)

        #ransac
        #plane_model, inline, outline=self.ransac_plane(downpcd, viz=False)
        #print(plane_model)


        # viz head model
        # mesh_c = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # mesh_c.scale(0.2, center=(0,0,0))

        # head_mesh = o3d.io.read_triangle_mesh("../stl/head_model.stl")
        # head_mesh.compute_vertex_normals()

        # # o3d.visualization.draw_geometries([head_mesh])

        # # head_pcd = head_mesh.sample_points_uniformly(number_of_points=500)
        # # o3d.visualization.draw_geometries([head_pcd])
        # # vis = o3d.visualization.Visualizer()
        # # vis.create_window()
        
        # head_pcd = head_mesh.sample_points_poisson_disk(number_of_points=500, init_factor=5)
        # R_h = head_pcd.get_rotation_matrix_from_xyz((np.pi/2, -np.pi/2, np.pi))

        # head_pcd.rotate(R_h)
        # head_pcd.rotate(obb.R)
        
        # head_pcd.translate(obb.get_center())
        # print(obb.extent)
        # head_pcd.translate(np.array([0,-obb.extent[1]/3,obb.extent[2]*4/5]))
    
        # print(obb.R)
        #R_h = head_pcd.get_rotation_matrix_from_xyz((np.pi/2 ,0, 0))

        #head_pcd.rotate(R_h)
        # R_h = head_pcd.get_rotation_matrix_from_xyz((0 , np.pi/2, 0))
        # head_pcd.rotate(R_h)
        #o3d.visualization.draw_geometries([head_pcd, downpcd, mesh_c, obb])

        # vis.add_geometry(head_pcd)
        # vis.add_geometry(downpcd)
        # vis.update_geometry(head_pcd)
        # vis.poll_events()
        # vis.update_renderer()

        # center, radius, inliers = self.ransac_sphere(downpcd)

        # inline = downpcd.select_by_index(inliers).paint_uniform_color([1, 0, 0])
        # outline = downpcd.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])

        # mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=radius/100)
        # mesh_circle.compute_vertex_normals()
        # mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
        # mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))

        # if self.viz == True:
        #     o3d.visualization.draw_geometries([outline, mesh_circle, inline],"ransac sphere")


if __name__=="__main__":
    rospy.init_node("trajectory_for_comb")
    app = TrajectoryHair()
    rospy.spin()

    # sys.path.append(".")
    # points = np.load("points_b.npy")[:, :3]
    # points = points[np.where(~np.isnan(points))[0]]
    # print(points)
    # #numpy型->open3d型
    # pointcloud1 = o3d.geometry.PointCloud()
    # pointcloud1.points = o3d.utility.Vector3dVector(points)
    # pointcloud1.paint_uniform_color([0.1,0.9, 0.1])
    # o3d.visualization.draw_geometries([pointcloud1])

    #mesh_in = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
    #vertices = np.asarray(mesh_in.vertices)
    #noise = 0.5
    #vertices += np.random.logistic(0, noise, size=vertices.shape)
    #mesh_in.vertices = o3d.utility.Vector3dVector(vertices) # vertice changed
    #mesh_in.compute_vertex_normals()
    #mesh_in.paint_uniform_color([0.1, 0.9, 0.1])
    #o3d.visualization.draw_geometries([mesh_in])
    #pcd_load = mesh_in.sample_points_uniformly(number_of_points=2000)
    #o3d.visualization.draw_geometries([mesh_in])
    
    #points = np.asarray(pcd_load.points)
    
    # sph = pyrsc.Sphere()
    
    # center, radius, inliers = sph.fit(points, thresh=0.4)
    # print("center: " + str(center))
    # print("radius: " + str(radius))



    # inline = pointcloud1.select_by_index(inliers).paint_uniform_color([1, 0, 0])
    # outline = pointcloud1.select_by_index(inliers, invert=True).paint_uniform_color([0, 1, 0])

    # mesh_circle = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    # mesh_circle.compute_vertex_normals()
    # mesh_circle.paint_uniform_color([0.9, 0.1, 0.1])
    # mesh_circle = mesh_circle.translate((center[0], center[1], center[2]))
    # o3d.visualization.draw_geometries([outline, mesh_circle, inline])

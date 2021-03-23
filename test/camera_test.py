# _*_ coding:utf-8 _*_
# @time: 2021/3/23 上午9:28
# @author: 张新新
# @email: 1262981714@qq.com

import open3d
import numpy as np
import cv2
from Module.camera import camera
import transforms3d as t3d
def camera_shape_create():
    '''
    生成相机的外形
    Returns:

    '''
    triangle_points = 0.05 * np.array([[1, 1, 2], [1, -1, 2], [-1, -1, 2],[-1, 1,2],[0, 0, 0]], dtype=np.float32)
    lines = [[0, 1], [1, 2], [2, 3],[0,3],[0,4],[1,4],[2,4],[3,4]]  # Right leg
    colors = [[0, 0, 1] for i in range(len(lines))]  # Default blue
    # 定义三角形的三个角点
    point_pcd = open3d.geometry.PointCloud()  # 定义点云
    point_pcd.points = open3d.Vector3dVector(triangle_points)

    # 定义三角形三条连接线
    line_pcd = open3d.LineSet()
    line_pcd.lines = open3d.Vector2iVector(lines)
    line_pcd.colors = open3d.Vector3dVector(colors)
    line_pcd.points = open3d.Vector3dVector(triangle_points)
    open3d.write_line_set("../data/testDate/a.ply",line_pcd)
    vis = open3d.Visualizer()
    vis.create_window(window_name='Open3D_1', width=600, height=600, left=10, top=10, visible=True)
    vis.get_render_option().point_size = 20

    vis.add_geometry(line_pcd)
    while True:
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        cv2.waitKey(100)

def camera_test_read_write():
    '''
    测试相机读写
    Returns:

    '''
    camera1 = camera()
    width, height, fx, fy, cx, cy = 640, 480, 450.2, 450.4, 316.3, 192.0
    camera1.intrinsic = open3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    camera1.write_camera_intrinsic("../config/camera_intrinsic.json")
def camera_test_capture():
    '''
    测试相机运动
    Returns:

    '''
    pcd = open3d.io.read_point_cloud('../data/testDate/8_input.ply')
    external_camera = open3d.io.read_pinhole_camera_parameters("../config/externalCamera.json")
    camera1 = camera()
    camera1.setVisble(True)
    width, height, fx, fy, cx, cy = 640, 480, 450.2, 450.4, 316.3, 293.5
    camera1.intrinsic = open3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    extrinsic = np.array([[1,0,0,0.0078125],
 [0,-1,-0,-0.0078125],
 [-0,-0,-1,0.87120328],
 [ 0,0,0,1]])
    camera1.setExtrisic(extrinsic)
    change = np.array([[1,0,0,0.01],
                       [0,1,0,0],
                       [0,0,1,0],
                       [0,0,0,1]])
    camera1.showCapture([pcd],"win1")
    camera1.isShowTrajectory=True
    camera1.showTrajectory(external_camera)
    camera1.setExtrisic(camera1.parameters.extrinsic)
    i=0
    while(True):
        print(i)
        i=i+1
        camera1.setExtrisic(np.dot(camera1.parameters.extrinsic,change))
        # camera1.updateCapture()
        camera1.updateTrajectory()
        cv2.waitKey(100)

    open3d.write_point_cloud("../data/testDate/a.ply",camera1.trajectoryShape)
    while(True):
        camera1.updateCapture()
        camera1.updateTrajectory()
def show_traj():
    pcl = open3d.io.read_point_cloud("../data/testDate/a.ply")
    vis = open3d.Visualizer()
    vis.create_window(window_name='Open3D_1', width=640, height=480, left=10, top=10, visible=True)
    vis.add_geometry(pcl)
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    open3d.write_pinhole_camera_parameters("../config/externalCamera.json",param)

    while True:
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        cv2.waitKey(100)

# show_traj()
camera_test_capture()






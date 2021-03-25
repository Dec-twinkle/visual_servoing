# _*_ coding:utf-8 _*_
# @time: 2021/3/22 下午4:20
# @author: 张新新
# @email: 1262981714@qq.com
from Module.module_base import module_base
import open3d as o3d
import numpy as np
import cv2
class camera(module_base):
    def __init__(self):
        self.intrinsic = None
        self.parameters = None
        self.trajectory = None
        self.samplingTime = None
        self.visible = False
        self.windowName = None
        self.shape = None
        self.initShape = None
        self.vis = None
        self.isShowTrajectory= False
        self.trajectoryShape = o3d.geometry.PointCloud()
        self.trajectoryVis = None
        self.isfixed = False
    def write_camera_intrinsic(self,filename):
        '''
        将相机内参写入文件
        Args:
            filename: intrinsic file path

        Returns:
        '''
        try:
            o3d.io.write_pinhole_camera_intrinsic(filename,self.intrinsic)
        except Exception:
            print("cannot write intrinsic to {}".format(filename))
            exit(-1)


    def read_camera_intrinsic(self,filename):
        '''
        从文件中读入相机内参
        Args:
            filename:

        Returns:open3d.camera.PinholeCameraParameters
        '''
        try:
            self.intrinsic = o3d.io.read_pinhole_camera_intrinsic(filename)
        except Exception:
            print("cannot read intrinsic from {}".format(filename))
            exit(-1)

    def write_camera_parameters(self,filename):
        '''
        将相机参数写入对于对应文件
        Args:
            filename: camera parameters filename

        Returns:

        '''
        try:
            o3d.io.write_pinhole_camera_parameters(filename,self.parameters)
        except Exception:
            print("cannot write parameters to {}".format(filename))
            exit(-1)

    def read_camera_parameters(self,filename):
        '''
        从文件中读入相机参数
        Args:
            filename: camera parameters filename

        Returns:

        '''
        try:
            self.parameters = o3d.io.read_pinhole_camera_parameters(filename)
        except Exception:
            print("cannot read parameters from {}".format(filename))
            exit(-1)

    def getShape(self):
        '''
        获取相机展示的形状
        Returns:
            open3d.lineset
        '''
        return self.shape

    def setShape(self,filename):
        '''
        设置外形
        Args:
            filename:

        Returns:

        '''
        try:
            self.initShape = o3d.io.read_line_set(filename)
            self.shape = o3d.io.read_line_set(filename)
        except Exception:
            print("cannot read parameters from {}".format(filename))
            exit(-1)




    def isVisble(self):
        '''
        相机是否可见
        Returns:bool:True 可见
                    false 不可见

        '''
        return self.visible

    def setVisble(self,isVisble):
        '''
        设置是否可见
        Args:
            isVisble: True:可见
                      false:不可见

        Returns:
        '''
        self.visible=isVisble

    def showCapture(self,scene,windows):
        '''
        展示相机看见的视野
        Args:
            scene:

        Returns:

        '''
        self.vis = o3d.Visualizer()
        self.vis.create_window(window_name=windows["windowsName"], width=windows["width"], height=windows["height"], left=windows["left"], top=windows["top"], visible=True)
        self.vis.get_render_option().point_size = 10
        for geo in scene:
            self.vis.add_geometry(geo)

    def setExtrisic(self,extrinsic):
        if self.parameters is None:
            if self.intrinsic is None:
                print("intrisic of camera is None")
                exit(-1)
            else:
                self.parameters = o3d.camera.PinholeCameraParameters()
                self.parameters.extrinsic = extrinsic
                self.parameters.intrinsic = self.intrinsic
        else:
            self.parameters.extrinsic = extrinsic
        if self.trajectory is None:
            self.trajectory = o3d.camera.PinholeCameraTrajectory()
        self.trajectory.parameters.append(self.parameters)
        if not self.vis is None:
            self.updateCapture()
        if self.isShowTrajectory:
            originInWord = np.dot(np.linalg.inv(self.parameters.extrinsic), np.array([[0, 0, 0, 1]]).T)
            if self.trajectoryShape.points is None:
                self.trajectoryShape.points = o3d.Vector3dVector(np.array([originInWord[0:3,0]]))
                self.trajectoryShape.colors = o3d.Vector3dVector(np.array([[0,0,1]]))
            else:
                points = np.asarray(self.trajectoryShape.points)
                colors = np.asarray(self.trajectoryShape.colors)
                points = np.append(points,np.array([originInWord[0:3,0]]),0)
                colors = np.append(colors,np.array([[0,0,1]]),0)
                pld = o3d.PointCloud()
                pld.points = o3d.utility.Vector3dVector(np.array([originInWord[0:3,0]]))
                pld.colors = o3d.utility.Vector3dVector(np.array([[0,0,1]]))
                self.trajectoryVis.add_geometry(pld)
                self.trajectoryShape.points = o3d.utility.Vector3dVector(points)
                self.trajectoryShape.colors = o3d.utility.Vector3dVector(colors)


    def updateCapture(self):
        '''
        更新相机的视野
        Args:
            scene:

        Returns:

        '''
        if self.isVisble():
            points = np.asarray(self.initShape.points)
            points = np.append(points,np.ones([points.shape[0],1]),1).T
            points = np.dot(np.linalg.inv(self.parameters.extrinsic),points).T
            self.shape.points = o3d.Vector3dVector(points[:,:3])
        ctr = self.vis.get_view_control()
        if not self.isfixed:
            param = ctr.convert_to_pinhole_camera_parameters()
            param.extrinsic = self.parameters.extrinsic
            ctr.convert_from_pinhole_camera_parameters(param)
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()


    def showTrajectory(self,windows):
        self.trajectoryVis = o3d.Visualizer()
        self.trajectoryVis.create_window(window_name=windows["windowsName"], width=windows["width"], height=windows["height"],
                               left=windows["left"], top=windows["top"], visible=True)
        self.trajectoryVis.get_render_option().point_size = 5
        axis_pcd = o3d.geometry.create_mesh_coordinate_frame(size=0.5, origin=[0, 0, 0])
        self.trajectoryVis.add_geometry(axis_pcd)

    def updateTrajectory(self):
        self.trajectoryVis.update_geometry()
        self.trajectoryVis.poll_events()
        self.trajectoryVis.update_renderer()









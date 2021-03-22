# _*_ coding:utf-8 _*_
# @time: 2021/3/22 下午4:20
# @author: 张新新
# @email: 1262981714@qq.com
from Module.module_base import module_base
import open3d as o3d
class camera(module_base):
    def __init__(self):
        self.intrinsic = None
        self.parameters = None
        self.trajectory = None
        self.velocity = None
        self.samplingTime = None
        self.visible = False
    def write_camera_intrinsic(self,filename):
        '''
        将相机内参写入文件
        Args:
            filename: intrinsic file path

        Returns:
        '''

    def read_camera_intrinsic(self,filename):
        '''
        从文件中读入相机内参
        Args:
            filename:

        Returns:open3d.camera.PinholeCameraParameters
        '''

    def write_camera_parameters(self,filename):
        '''
        将相机参数写入对于对应文件
        Args:
            filename: camera parameters filename

        Returns:

        '''

    def read_camera_parameters(self,filename):
        '''
        从文件中读入相机参数
        Args:
            filename: camera parameters filename

        Returns:

        '''

    def getShape(self):
        '''
        获取相机展示的形状
        Returns:
            open3d.lineset
        '''

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

    def showCapture(self,scene):
        '''
        展示相机看见的视野
        Args:
            scene:

        Returns:

        '''







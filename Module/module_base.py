# _*_ coding:utf-8 _*_
# @time: 2021/3/22 下午4:05
# @author: 张新新
# @email: 1262981714@qq.com
from abc import ABCMeta,abstractmethod

class module_base(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def getShape(self):
        '''
        获取物体的位置形状
        Returns: open3d.geometry

        '''
        pass

    @abstractmethod
    def isVisble(self):
        '''
        if the object is visible
        Returns:

        '''
        pass
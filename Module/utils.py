# _*_ coding:utf-8 _*_
# @time: 2021/3/25 下午6:47
# @author: 张新新
# @email: 1262981714@qq.com
import transforms3d as t3d
import numpy as np
def matrix2pose(matrix):
    euler = t3d.euler.mat2euler(matrix[:3, :3])
    return np.array([matrix[0,3],matrix[1,3],matrix[2,3],euler[0],euler[1],euler[2]])

def pose2matrix(pose):
    matrix  = np.eye(4)
    matrix[:3,:3] = t3d.euler.euler2mat(pose[3],pose[4],pose[5])
    matrix[:3,3] = pose[:3]
    return matrix
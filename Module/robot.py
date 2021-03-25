# _*_ coding:utf-8 _*_
# @time: 2021/3/24 下午3:35
# @author: 张新新
# @email: 1262981714@qq.com
import numpy as np
import cv2
import time
import transforms3d as t3d
import _thread
class Robot():
    def __init__(self):
        self.samplingTime = 0
        self.position = None
        self.velocity = np.array([0,0,0,0,0,0])
        self.end = True
        self.frame = np.eye(4)

    def setSamplingtime(self,samplingTime):
        '''
        设置采样时间
        Args:
            samplingTime:

        Returns:

        '''
        self.samplingTime = samplingTime
    def setFrame(self,frame):
        '''
        设置参考坐标系
        Args:
            frame:

        Returns:

        '''
        self.frame = frame

    def setPosition(self,position):
        '''
        设置机械臂末端坐标系
        Args:
            position:

        Returns:

        '''
        if np.size(position)==16:
            self.position=position
        elif np.size(position)==6:
            self.position = np.eye(4)
            self.position[:3,:3] = t3d.euler.euler2mat(position[3],position[4],position[5])
            self.position[:3,3] = position[:3]
        else:
            print("error formulate")
            exit(-1)
        # self.position = position
    def getPosition(self):
        '''
        获取机器臂末端位置
        Returns:

        '''
        return self.position
        # euler = t3d.euler.mat2euler(self.position[:3,:3])
        # return np.array([self.position[0,3],self.position[1,3],self.position[2,3],euler[0],euler[1],euler[2]])
    def setVelocity(self,velocity):
        '''
        设置机械臂末端移动速度
        Args:
            velocity:

        Returns:

        '''
        self.velocity = velocity

    def run(self):
        def robot_run():
            while(self.end):
                positionInframe = np.dot(np.linalg.inv(self.frame),self.position)
                eulerInframe = t3d.euler.mat2euler(positionInframe[:3,:3])
                nextPositionInframe = np.eye(4)
                nextPositionInframe[:3,:3] = t3d.euler.euler2mat(eulerInframe[0]+self.velocity[3]*self.samplingTime,
                                                                 eulerInframe[1]+self.velocity[4]*self.samplingTime,
                                                                 eulerInframe[2]+self.velocity[5]*self.samplingTime)
                nextPositionInframe[:3,3] = positionInframe[:3,3]+self.velocity[:3] * self.samplingTime
                self.position = np.dot(self.frame,nextPositionInframe)
                # print("robotpose:",self.getPosition())
                time.sleep(self.samplingTime)

        _thread.start_new_thread(robot_run, ())

    def end(self):
        '''
        机械臂停止运行
        Returns:

        '''
        self.end = False

# robot = Robot()
# robot.setSamplingtime(0.04)
# robot.setPosition(np.array([0,0,0,0,0,0]))
# robot.frame = robot.position
# robot.setVelocity(np.array([0,0.1,0.1,0.1,0,0]))
# robot.run()
# time.sleep(robot.samplingTime*100)










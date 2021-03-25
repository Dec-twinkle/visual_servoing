# _*_ coding:utf-8 _*_
# @time: 2021/3/25 下午3:18
# @author: 张新新
# @email: 1262981714@qq.com
from Module.servo import Task
from Module.camera import camera
from Module.feature import FeaturePoint
from Module.robot import Robot
from Module.window import windowUtils
from Module import utils
import numpy as np
import transforms3d as t3d
import open3d as o3d
import time

def testVelocityTransform():
    servo = Task()
    Vc = np.array([0.1,0.2,0.3,0.01,0.02,0.03])
    eMc = np.eye(4)
    eMc[:3,:3] = t3d.euler.euler2mat(3.1415926,0,0)
    eMc[0,3] = 0.01
    servo.setHandeye(eMc)
    curCamerapose= servo.getCameraPoseFromRobot(np.eye(4))
    Ve = servo.transformCameraToRobotVelocity(Vc)
    Ve_change = np.eye(4)
    Ve_change[:3,:3] = t3d.euler.euler2mat(Ve[3],Ve[4],Ve[5])
    Ve_change[:3,3] = Ve[:3]
    Next_robot_pose = np.dot(np.eye(4),Ve_change)
    Next_camera_pose = servo.getCameraPoseFromRobot(Next_robot_pose)
    Vc_change = np.dot(np.linalg.inv(curCamerapose),Next_camera_pose)
    print("Ve_change:",t3d.euler.mat2euler(Vc_change[:3,:3]),Vc_change[:3,3])
    print("cur：",t3d.euler.mat2euler(curCamerapose[:3,:3]),curCamerapose[:3,3])
    print("next：",t3d.euler.mat2euler(Next_camera_pose[:3,:3]),Next_camera_pose[:3,3])

def testCalculateV():
    def projectFeature(point,camera):
        features = []
        for i in range(point.shape[0]):
            point_coord = np.ones([4,1])
            point_coord[:3,0] = point[i,:]
            # camera = camera()
            point_coord = np.dot(camera.parameters.extrinsic,point_coord)
            Z = point_coord[2,0]
            features.append(FeaturePoint(point_coord[0,0]/Z,point_coord[1,0]/Z,Z))
        return features
    cam = camera()
    cam2 = camera()
    points = np.array([[-0.1, -0.1, 0],
                       [0.1, -0.1, 0],
                       [0.1, 0.1, 0],
                       [-0.1, 0.1, 0]])
    colors = np.array([[0,0,1],
                      [0,1,0],
                       [1,0,0],
                       [0,0,0]])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.Vector3dVector(points)
    pcd.colors = o3d.Vector3dVector(colors)
    wins = windowUtils()
    destExtrinsic = np.eye(4)

    destExtrinsic[2,3] = 0.75
    cam.read_camera_parameters("../config/externalCamera.json")
    cam2.read_camera_parameters("../config/externalCamera.json")
    cam2.isfixed = False
    cam.isfixed = False
    cam.showCapture([pcd],wins.getWindow("cur"))
    cam2.showCapture([pcd],wins.getWindow("dest"))
    cam2.setExtrisic(destExtrinsic)
    curCMo = np.eye(4)
    curCMo[:3,:3] = t3d.euler.euler2mat(10*3.1415926/180,-10*3.1415926/180,50*3.1415926/180)
    curCMo[0,3] = 0.15
    curCMo[1,3] = -0.1
    curCMo[2,3] = 1
    cam.setExtrisic(curCMo)
    destFeature = projectFeature(points, cam2)
    servoTask = Task()
    servoTask.setLambda(0.5)
    servoTask.setHandeye(np.eye(4))
    servoTask.setDFeature(destFeature)
    robot1 = Robot()
    robot1.setSamplingtime(0.1)
    robot1.setPosition(cam.parameters.extrinsic)
    robot1.setFrame(robot1.getPosition())
    robot1.run()
    i=0
    extrinsic_before = None
    while(i<2):
        i=i+1
        robot1.setFrame(robot1.getPosition())
        extrinsic_before = cam.parameters.extrinsic
        cam.setExtrisic(servoTask.getCameraPoseFromRobot(robot1.getPosition()))
        extrinsic_after = cam.parameters.extrinsic
        print(utils.matrix2pose(np.dot(np.linalg.inv(extrinsic_before),extrinsic_after)))
        curFeature = projectFeature(points,cam)
        servoTask.setCFeature(curFeature)
        vc = servoTask.getCameraVelocity()
        # print("error",servoTask.getError())
        # print("le",servoTask.getInteractionMatrix())
        print("vc:",vc)
        ve = servoTask.transformCameraToRobotVelocity(vc)
        robot1.setVelocity(ve)
        cam.updateCapture()
        cam2.updateCapture()
        time.sleep(robot1.samplingTime*10)










testCalculateV()


#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy, sys
import math
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
from aruco_msgs.msg import Marker

import pyrealsense2 as rs
import cv2.aruco as aruco
import cv2
import numpy as np
import time
import os


pi=math.pi
pi2=pi/2
rad2angle=180/pi


# 创建 RealSense 管道
pipeline = rs.pipeline()

# 创建配置对象
config = rs.config()

# 启用 RGB 和深度流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 开始管道
pipeline.start(config)

#相机参数，至少为1
camera_matrix = np.array([[863.439262, 0, 645.612360], 
                          [0, 858.739541, 406.812029], 
                          [0, 0, 1]], dtype=float)

# 如果没有畸变，也可以使用全零数组
dist_coeffs = np.array([0.048015,-0.083847,0.011333,0.007398,0.000000], dtype=float)


#设置预定义的字典
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)


def get_aruco_pose(delay):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        exit()

        # 将图像转换为 numpy 数组
    frame = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())



    #灰度化
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #使用默认值初始化检测器参数
    parameters =  aruco.DetectorParameters()


    #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    #画出标志位置
    aruco.drawDetectedMarkers(frame, corners,ids)

    # print(corners)


    if ids is not None:
        # 估计姿态 (假设每个标记边长为0.05米)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.027, camera_matrix, dist_coeffs)
        # 旋转矩阵 平移矩阵 计算内部的标记角点


        # 绘制检测到的标记和位姿
        for i in range(len(ids)):
            frame=cv2.aruco.drawDetectedMarkers(frame, corners)  # 绘制标记的边框
            frame=cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)  # 绘制坐标轴
            
            # 输出位姿信息
            print(f"Marker ID: {ids[i][0]}")
            print(f"Rotation Vector (rvec): {rvecs[i].flatten()}")
            print(f"Translation Vector (tvec): {tvecs[i].flatten()}")

        tv=tvecs[0].flatten()
        tv = [float('{:.4f}'.format(i)) for i in tv]
        text=str(tv)
        cv2.putText(frame, text, (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 255, 0), 2)
        print(tv[0],tv[1],tv[2])

        return tv
    # 显示 RGB 图像
    cv2.imshow('RGB Image', frame)

    # 按 'q' 键退出
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        exit()
    
def save_image():
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    frame = np.asanyarray(color_frame.get_data())

    # 计算文件夹里jpg文件数量，以便于关闭软件后重新打开采集不会将已有图片覆盖
    count = 0
    # 确保目录存在，如果不存在则创建
    save_dir = '../imgs/'
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    for filename in os.listdir('../imgs/'):
        if filename.endswith('.jpg'):
            count += 1
        # print(count)
    cv2.imshow('RGB Image', frame)
    cv2.waitKey(100)

    # 保存图像，保存到上一层的imgs文件夹内，以1、2、3、4...为文件名保存图像
    cv2.imwrite('../imgs/{}.jpg'.format(count + 1), frame)


class MoveItDrawStarDemo:

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('sagittarius_arm')
        gripper = MoveGroupCommander('sagittarius_gripper')

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
        gripper.set_pose_reference_frame(reference_frame) #夹爪
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)
        arm.set_goal_joint_tolerance(0.02)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1.0)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()



        # # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()



        dist={
            "x":0,"y":1,"z":2,"rol":3,"pit":4,"yaw":5,"grab":6,"delay":7,"capture":8
        }
        trail=[

            # [0.23,  0 , 0.2 ,  0,  0  , 0,   0,  5], #标定组1
            # [0.23, 0.1, 0.2 ,  0,  0, 0,   0,  5], #标定组2
            # [0.23,-0.1, 0.25,  0,  0  , 0,   0,  5], #标定组3
            # [0.3 ,-0.1, 0.25,  0,  0.25  , 0,   0,  5], #标定组4
            # [0.3 ,-0.1, 0.25,  0.25,  0.25  , 0,   0,  5], #标定组5
            # [0.3 ,-0.1, 0.25,  0.25,  0.25  , 0.25,   0,  5], #标定组6
            # [0.3 ,-0.1, 0.3 ,  0.25,  0.25  , -0.25,   0,  5], #标定组7
            # [0.3 ,-0.2, 0.3 ,  0.25,  0.25  , 0,   0,  5], #标定组8
            # [0.35,-0.1, 0.15,  0,  0  , 0,   0,  5], #标定组9
            # [0.4 ,-0.1, 0.15,  0,  0  , 0,   0,  5], #标定组10


            # [0.2 , 0.2 , 0.2 ,  0,  0  , -0.2,   0,  5], #标定组1
            # [0.2 , 0.1 , 0.2 ,  0,  0  , -0.1,   0,  5], #标定组2
            # [0.2 , 0   , 0.2 ,  pi2,  0  , 0,   0,  5], #标定组3
            # [0.2 , -0.1, 0.2 ,  0,  0  , 0.1,   0,  5], #标定组4
            # [0.2 , -0.2, 0.2 ,  0,  0  , 0.2,   0,  5], #标定组5

            # [0.3 , -0.2, 0.2 ,  0,  0.2  , 0.2,   0,  5], #标定组1
            # [0.3 , -0.1, 0.2 ,  0,  0.2  , 0.1,   0,  5], #标定组2
            # [0.3 , 0   , 0.2 ,  -pi2,  0.2  , 0  ,   0,  5], #标定组3
            # [0.3 ,  0.1, 0.2 ,  0,  0.2  , -0.1,   0,  5], #标定组4
            # [0.3 ,  0.2, 0.2 ,  0,  0.2  , -0.2,   0,  5], #标定组5

            # [0.2 , 0.2 , 0.25 ,  0,  0  , -0.2,   0,  5], #标定组1
            # [0.2 , 0.1 , 0.25 ,  0,  0  , -0.1,   0,  5], #标定组2
            # [0.2 , 0   , 0.25 ,  pi2,  0  , 0,   0,  5], #标定组3
            # [0.2 , -0.1, 0.25 ,  0,  0  , 0.1,   0,  5], #标定组4
            # [0.2 , -0.2, 0.25 ,  0,  0  , 0.2,   0,  5], #标定组5

            #x,y,z,roll,pitch,yaw,grab,delay
            [0.2,0, 0.2,  0,  pi2, 0,   1,  0], #到达瓶盖处
            # [0.25,0, 0.1,  pi2,  pi2, 0,   1,  0], #
            # [0.25,0, 0.2,  pi2,  pi2, 0,   1,  0], #
            # [0.25,-0.2, 0.1,  0,  pi2, 0,   0,  0], #
            # [0.25,-0.2, 0.1,  0,  pi2, 0,   0,  0], #
        ]





        target_pose_base = PoseStamped()
        target_pose_base.header.frame_id = reference_frame
        target_pose_base.header.stamp = rospy.Time.now()


        for i in range(len(trail)):
            
            target_pose_base.pose.position.x = trail[i][dist["x"]]
            target_pose_base.pose.position.y = trail[i][dist["y"]]
            target_pose_base.pose.position.z = trail[i][dist["z"]]

            roll =  trail[i][dist["rol"]]    #腕关节
            pitch = trail[i][dist["pit"]]    #朝下
            yaw =   trail[i][dist["yaw"]]    #转轴

            rotation=R.from_euler("xyz",[roll,pitch,yaw])
            quaternion=rotation.as_quat()
            # 设置目标姿态

            target_pose_base.pose.orientation.x = quaternion[0]
            target_pose_base.pose.orientation.y = quaternion[1]
            target_pose_base.pose.orientation.z = quaternion[2]
            target_pose_base.pose.orientation.w = quaternion[3]

            # 设置机械臂终端运动的目标位姿
            arm.set_pose_target(target_pose_base, end_effector_link)
            arm.go()


            if(trail[i][dist["grab"]]):
                # 控制机械臂夹爪抓住笔
                gripper.set_joint_value_target([-0.0197, -0.0197])
                gripper.go()
            else:
                # 松开笔
                gripper.set_named_target('open')
                gripper.go()



            # save_image()
            rospy.sleep(1)

            rospy.sleep(int(trail[i][dist["delay"]]))






        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

       # 控制机械臂先回到初始化位置
        arm.set_named_target('sleep')
        arm.go()
        rospy.sleep(1)
        
        # 松开笔
        gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(2)

  
        
        # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)





if __name__ == "__main__":
    try:
        # 初始化ROS节点
        rospy.init_node('moveit_draw_star_demo', anonymous=True)
        rospy.wait_for_message("/start_topic", String)
        MoveItDrawStarDemo()
        # listener()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3.8
# _*_ coding: utf-8 _*_

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation as R

pi=3.1415926
pi2=pi/2

dist={
    "x":0,"y":1,"z":2,
    "r":3,"p":4,"y":5,
    "delay":6
}
trail=[
    #x,y,z,roll,yaw,pitch
    [0.06,-0.0,0.5,   0,  0, 0,2], #到达瓶盖处
    # [0,-0.4,0.2,   0,  pi, pi2,1],#旋转开盖
    # [0,-0.4,0.2,   0,  pi,-pi2,1],#旋转开盖
    # [0,-0.4,0.25,  0,  pi,-pi2,1],#放置瓶盖1
    # [0,-0.2,0.2,   0,  pi,-pi2,1],#放置瓶盖2
    # [0,-0.2,0.1,   0,  pi,-pi2,1],#放置瓶盖3
    # [0,-0.3,0.2,   pi*2/3, -pi2,-pi2,1],#抓瓶子
]


def move_arm_to_grasp_target():

    # 初始化moveit_commander和rospy节点
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('workflow', anonymous=True)
    # 初始化需要使用的对象

    group_name = "arm" # 替换为你的机械臂group名称

    group = moveit_commander.MoveGroupCommander(group_name)


    # 设置目标位置1
    target_pose = geometry_msgs.msg.Pose()

    for i in range(len(trail)):
        target_pose.position.x = trail[i][0] 
        target_pose.position.y = trail[i][1]
        target_pose.position.z = trail[i][2]


        roll =  trail[i][3]    #腕关节
        pitch = trail[i][4]    #朝下
        yaw =   trail[i][5]    #转轴

        rotation=R.from_euler("xyz",[roll,pitch,yaw])
        quaternion=rotation.as_quat()
        # quaternion=[0.75,-0.65,0,0.4]
        # 设置目标姿态
        target_pose.orientation.x = quaternion[0] #
        target_pose.orientation.y = quaternion[1] #
        target_pose.orientation.z = quaternion[2] #
        target_pose.orientation.w = quaternion[3] #

        rospy.loginfo(target_pose)

        rotation_q=R.from_quat(quaternion)
        euler_angle=rotation_q.as_euler("xyz",degrees=False)
        rospy.loginfo(euler_angle)

        group.set_pose_target(target_pose)
        # 规划并执行路径
        plan = group.go()
        info="step "+str(i+1)+" over"
        rospy.loginfo(info)
        rospy.sleep(trail[i][6])



    group.stop() # 停止所有剩余的运动
    group.clear_pose_targets()



    # 关闭moveit
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':

    try:
        move_arm_to_grasp_target()
    except rospy.ROSInterruptException:
        raise
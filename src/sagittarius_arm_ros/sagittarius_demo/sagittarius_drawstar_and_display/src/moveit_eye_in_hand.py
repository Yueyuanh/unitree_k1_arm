#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
import math

pi=math.pi
pi2=pi/2


class MoveItDrawStarDemo:
    waypoints = []

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



#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()



        dist={
            "x":0,"y":1,"z":2,"rol":3,"pit":4,"yaw":5,"grab":6,"delay":7
        }
        trail=[

            # [0.23,  0, 0.2,  0,  0  , 0,   0,  5], #标定组1

            # [0.23,0.1, 0.2,  0,  0, 0,   0,  5], #标定组2

            # [0.23,-0.1, 0.25,  0,  0  , 0,   0,  5], #标定组3

            # [0.3,-0.1, 0.25,  0,  0.25  , 0,   0,  5], #标定组4

            # [0.3,-0.1, 0.25,  0.25,  0.25  , 0,   0,  5], #标定组5

            # [0.3,-0.1, 0.25,  0.25,  0.25  , 0.25,   0,  5], #标定组6

            # [0.3,-0.1, 0.3,  0.25,  0.25  , -0.25,   0,  5], #标定组7

            # [0.3,-0.2, 0.3,  0.25,  0.25  , 0,   0,  5], #标定组8

            # [0.35,-0.1, 0.15,  0,  0  , 0,   0,  5], #标定组9

            # [0.4,-0.1, 0.15,  0,  0  , 0,   0,  5], #标定组10

            [0.23,  0, 0.2,  1,  0  , 0,   0,  5], #标定组1
            [0.23,  0, 0.2,  0,  1  , 0,   0,  5], #标定组1
            [0.23,  0, 0.2,  0,  0  , 1,   0,  5], #标定组1


            #x,y,z,roll,pitch,yaw,grab,delay
            # [0.25,0, 0.1,  0,  pi2, 0,   1,  0], #到达瓶盖处
            # [0.25,0, 0.1,  pi2,  pi2, 0,   1,  0], #
            # [0.25,0, 0.2,  pi2,  pi2, 0,   1,  0], #
            # [0.25,-0.2, 0.1,  0,  pi2, 0,   0,  0], #
            # [0.25,-0.2, 0.1,  0,  pi2, 0,   0,  0], #
        ]





        # 设置五角星基准点的位置
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
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        # 初始化ROS节点
        rospy.init_node('moveit_draw_star_demo', anonymous=True)

        rospy.wait_for_message("/start_topic", String)
        MoveItDrawStarDemo()
    except rospy.ROSInterruptException:
        pass

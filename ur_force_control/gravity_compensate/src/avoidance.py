#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy
 
class MoveAttachedObjectDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_attached_object_demo')
        
        # 初始化场景对象，用来监听外部环境的变化
        scene = PlanningSceneInterface()
        rospy.sleep(1)        

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 移除场景中之前运行残留的物体
       # 移除场景中之前运行残留的物体
        scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 
        scene.remove_world_object('target')
 
        # 设置桌面的高度
        table_ground = 0.6
        
        # 设置table和tool的三维尺寸
        table_size = [3.0, 3.0, 0.02]  #设置长宽高
        tool_size = [0.2, 0.02, 0.02]
        
        # # 设置tool的位姿
        # p = PoseStamped() 

        # #tool放置位置的参考坐标系
        # p.header.frame_id = end_effector_link
        
        # #tool的具体位置姿态
        # p.pose.position.x = tool_size[0] / 2.0 - 0.025
        # p.pose.position.y = -0.015
        # p.pose.position.z = 0.0
        # p.pose.orientation.x = 0
        # p.pose.orientation.y = 0
        # p.pose.orientation.z = 0
        # p.pose.orientation.w = 1
        
        # # 将tool附着到机器人的终端，该函数为附着某物体到机器人上
        # scene.attach_box(end_effector_link, 'tool', p, tool_size)
 
        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -0.115
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)  #添加障碍物
        
        rospy.sleep(2)  
 
        # # 更新当前的位姿
        # arm.set_start_state_to_current_state()
 
        # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.827228546495185, 0.29496592875743577, 1.1185644936946095, -0.7987583317769674, -0.18950024740190782, 0.11752152218233858]
        # arm.set_joint_value_target(joint_positions)
                 
        # # 控制机械臂完成运动
        # arm.go()
        # rospy.sleep(1)
        
        # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
 
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveAttachedObjectDemo()

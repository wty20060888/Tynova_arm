#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gripper')
        
        # 设置Tynova机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
        
        # 控制Tynova机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)
         
        # 设置Tynova夹爪的目标位置，并控制夹爪运动
        gripper.set_joint_value_target([0.01])
        gripper.go()
        rospy.sleep(1)
         
        # first设置Tynova机械臂的目标位置，使用11轴的位置数据进行描述（单位：弧度）
        joint_positions = [0, -20, -24, -21, 14, 47, 12, 30, 36, 28, 0]
        x_ = []
        for i in joint_positions:
            x_.append(i * 3.1415 /180)
        arm.set_joint_value_target(x_)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        # 2nd设置Tynova机械臂的目标位置，使用11轴的位置数据进行描述（单位：弧度）
        joint_positions = [0, -20, -24, -25, 20, 58, 13, 30, 36, 28, 0]
        x_ = []
        for i in joint_positions:
            x_.append(i * 3.1415 /180)
        arm.set_joint_value_target(x_)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        # 3rd设置Tynova机械臂的目标位置，使用11轴的位置数据进行描述（单位：弧度）
        joint_positions = [0, -20, -24, -32, 19, 61, 13, 32, 36, 28, 0]
        x_ = []
        for i in joint_positions:
            x_.append(i * 3.1415 /180)
        arm.set_joint_value_target(x_)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

        # lastpath设置Tynova机械臂的目标位置，使用11轴的位置数据进行描述（单位：弧度）
        joint_positions = [0, -37, -9, -50, -2, 85, 29, 17, 2, 43, -90]
        x_ = []
        for i in joint_positions:
            x_.append(i * 3.1415 /180)
        arm.set_joint_value_target(x_)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass

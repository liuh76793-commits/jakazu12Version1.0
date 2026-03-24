#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import math
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def main():
    # 结构拆解 1：节点与接口初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('jaka_test_node', anonymous=True)

    # 结构拆解 2：绑定规划组
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # 结构拆解 3：注入安全测试空间变量
    target_pose = Pose()
    # 设定空间绝对位置 (单位：米)
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.4

    # 设定空间绝对姿态 (欧拉角转四元数：Roll=180度, Pitch=0, Yaw=0，使末端朝下)
    q = quaternion_from_euler(math.radians(180.0), 0.0, 0.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    rospy.loginfo("开始规划机械臂至测试空间位置...")
    
    # 结构拆解 4：执行机械臂运动学规划与底层下发
    arm_group.set_pose_target(target_pose)
    arm_success = arm_group.go(wait=True)
    
    # 清理规划残余
    arm_group.stop()
    arm_group.clear_pose_targets()

    # 结构拆解 5：级联执行夹爪动作
    if arm_success:
        rospy.loginfo("机械臂到达目标，开始闭合夹爪...")
        # 设定夹爪闭合的关节变量 (0.4 弧度约为半闭合状态)
        gripper_group.set_joint_value_target([0.4])
        gripper_group.go(wait=True)
        gripper_group.stop()
        rospy.loginfo("测试流程全链路执行完毕。")
    else:
        rospy.logerr("机械臂逆运动学求解失败，无法到达指定空间位置。")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
测试脚本：验证轨迹桥接功能
发布测试轨迹消息到 FUEL 系统，检查 agilicious 参考轨迹是否正确生成
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, Vector3, Quaternion
from quadrotor_msgs.msg import PositionCommand
from agiros_msgs.msg import Reference, Setpoint, QuadState, Command
from nav_msgs.msg import Path, PoseStamped

class TrajectoryBridgeTester:
    def __init__(self):
        rospy.init_node('trajectory_bridge_tester', anonymous=True)
        
        # 发布器
        self.pos_cmd_pub = rospy.Publisher('planning/pos_cmd', PositionCommand, queue_size=10)
        self.trajectory_pub = rospy.Publisher('planning/trajectory', Path, queue_size=10)
        
        # 订阅器
        self.reference_sub = rospy.Subscriber('agilicious/reference', Reference, self.reference_callback)
        
        # 测试参数
        self.test_duration = 10.0  # 秒
        self.publish_rate = 20.0   # Hz
        self.test_type = "circle"  # "circle", "eight", "line"
        
        # 状态
        self.received_references = []
        self.test_start_time = None
        
        rospy.loginfo("轨迹桥接测试器已初始化")
        rospy.loginfo("测试类型: %s", self.test_type)
        rospy.loginfo("测试时长: %.1f 秒", self.test_duration)
        
    def reference_callback(self, msg):
        """接收 agilicious 参考轨迹的回调函数"""
        self.received_references.append(msg)
        rospy.loginfo("收到参考轨迹，包含 %d 个点", len(msg.points))
        
        # 打印第一个点的信息
        if msg.points:
            first_point = msg.points[0]
            rospy.loginfo("第一个点 - 位置: (%.2f, %.2f, %.2f), 时间: %.2f", 
                         first_point.state.pose.position.x,
                         first_point.state.pose.position.y,
                         first_point.state.pose.position.z,
                         first_point.state.t)
    
    def generate_circle_trajectory(self, t):
        """生成圆形轨迹"""
        radius = 2.0
        height = 1.5
        omega = 2 * math.pi / self.test_duration
        
        x = radius * math.cos(omega * t)
        y = radius * math.sin(omega * t)
        z = height
        
        vx = -radius * omega * math.sin(omega * t)
        vy = radius * omega * math.cos(omega * t)
        vz = 0.0
        
        ax = -radius * omega * omega * math.cos(omega * t)
        ay = -radius * omega * omega * math.sin(omega * t)
        az = 0.0
        
        yaw = omega * t
        yaw_dot = omega
        
        return x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot
    
    def generate_eight_trajectory(self, t):
        """生成8字形轨迹"""
        scale = 1.5
        height = 1.5
        omega = 2 * math.pi / self.test_duration
        
        x = scale * math.sin(omega * t)
        y = scale * math.sin(omega * t) * math.cos(omega * t)
        z = height
        
        vx = scale * omega * math.cos(omega * t)
        vy = scale * omega * (math.cos(omega * t)**2 - math.sin(omega * t)**2)
        vz = 0.0
        
        ax = -scale * omega * omega * math.sin(omega * t)
        ay = -4 * scale * omega * omega * math.sin(omega * t) * math.cos(omega * t)
        az = 0.0
        
        yaw = math.atan2(vy, vx) if abs(vx) > 0.001 or abs(vy) > 0.001 else 0.0
        yaw_dot = 0.0  # 简化处理
        
        return x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot
    
    def generate_line_trajectory(self, t):
        """生成直线轨迹"""
        start_pos = [-2.0, -2.0, 1.0]
        end_pos = [2.0, 2.0, 1.0]
        
        alpha = t / self.test_duration
        alpha = max(0.0, min(1.0, alpha))
        
        x = start_pos[0] + alpha * (end_pos[0] - start_pos[0])
        y = start_pos[1] + alpha * (end_pos[1] - start_pos[1])
        z = start_pos[2] + alpha * (end_pos[2] - start_pos[2])
        
        vx = (end_pos[0] - start_pos[0]) / self.test_duration
        vy = (end_pos[1] - start_pos[1]) / self.test_duration
        vz = (end_pos[2] - start_pos[2]) / self.test_duration
        
        ax = 0.0
        ay = 0.0
        az = 0.0
        
        yaw = math.atan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0])
        yaw_dot = 0.0
        
        return x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot
    
    def publish_position_command(self, t):
        """发布位置命令"""
        if self.test_type == "circle":
            x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot = self.generate_circle_trajectory(t)
        elif self.test_type == "eight":
            x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot = self.generate_eight_trajectory(t)
        else:  # line
            x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot = self.generate_line_trajectory(t)
        
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        
        cmd.acceleration.x = ax
        cmd.acceleration.y = ay
        cmd.acceleration.z = az
        
        cmd.yaw = yaw
        cmd.yaw_dot = yaw_dot
        
        # 设置增益参数
        cmd.kx = [1.0, 1.0, 1.0]
        cmd.kv = [1.0, 1.0, 1.0]
        
        self.pos_cmd_pub.publish(cmd)
    
    def publish_trajectory_path(self):
        """发布轨迹路径"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"
        
        num_points = 50
        for i in range(num_points):
            t = i * self.test_duration / num_points
            
            if self.test_type == "circle":
                x, y, z, _, _, _, _, _, _, yaw, _ = self.generate_circle_trajectory(t)
            elif self.test_type == "eight":
                x, y, z, _, _, _, _, _, _, yaw, _ = self.generate_eight_trajectory(t)
            else:  # line
                x, y, z, _, _, _, _, _, _, yaw, _ = self.generate_line_trajectory(t)
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # 转换偏航角为四元数
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            
            path.poses.append(pose)
        
        self.trajectory_pub.publish(path)
        rospy.loginfo("发布轨迹路径，包含 %d 个点", len(path.poses))
    
    def run_test(self):
        """运行测试"""
        rospy.loginfo("开始轨迹桥接测试...")
        
        # 等待发布器和订阅器准备就绪
        rospy.sleep(1.0)
        
        # 发布轨迹路径
        self.publish_trajectory_path()
        
        # 开始发布位置命令
        rate = rospy.Rate(self.publish_rate)
        self.test_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = (rospy.Time.now() - self.test_start_time).to_sec()
            
            if current_time > self.test_duration:
                break
            
            self.publish_position_command(current_time)
            rate.sleep()
        
        rospy.loginfo("测试完成！")
        rospy.loginfo("总共收到 %d 个参考轨迹", len(self.received_references))
        
        # 分析结果
        if self.received_references:
            total_setpoints = sum(len(ref.points) for ref in self.received_references)
            rospy.loginfo("总共包含 %d 个设定点", total_setpoints)
            
            # 计算平均频率
            if len(self.received_references) > 1:
                time_span = (self.received_references[-1].header.stamp - 
                           self.received_references[0].header.stamp).to_sec()
                avg_freq = len(self.received_references) / time_span
                rospy.loginfo("平均发布频率: %.2f Hz", avg_freq)
        
        rospy.loginfo("测试结束")

if __name__ == '__main__':
    try:
        tester = TrajectoryBridgeTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
    except Exception as e:
        rospy.logerr("测试出错: %s", str(e)) 
#!/usr/bin/env python3
import rospy
import math
import threading
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        
        # 创建发布者
        self.cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=10)
        
        # 设置发布频率 (20Hz)
        self.rate = rospy.Rate(20) 
        
        # 轨迹参数
        self.lock = threading.Lock()
        with self.lock:  # 初始化时也加锁
            self.start_time = rospy.Time.now()
            self.trajectory_type = "waiting"  # 初始等待状态

        # 初始化消息
        self.cmd_msg = PositionCommand()
        self.cmd_msg.kx = [5.0, 5.0, 5.0]  # 位置增益
        self.cmd_msg.kv = [0.4, 0.4, 0.4]  # 速度增益
        self.cmd_msg.trajectory_id = 1
        self.cmd_msg.trajectory_flag = 1

        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def keyboard_listener(self):
        """独立线程监听键盘输入"""
        while not rospy.is_shutdown():
            try:
                user_input = input("\n请选择轨迹类型: \n"
                                  "1 - 圆形轨迹\n"
                                  "2 - 直线轨迹\n"
                                  "3 - 定点 (0,0,3)\n"
                                  "4 - 定点 (1,1,3)\n"
                                  "输入选项 (1-4): ")
                
                with self.lock:
                    if user_input == '1':
                        self.trajectory_type = "circle"
                        print("切换到圆形轨迹")
                    elif user_input == '2':
                        self.trajectory_type = "line"
                        print("切换到直线轨迹")
                    elif user_input == '3':
                        self.trajectory_type = "hover_origin"
                        print("切换到原点悬停")
                    elif user_input == '4':
                        self.trajectory_type = "hover_corner"
                        print("切换到角落悬停")
                    else:
                        print("无效输入，请重新输入1-4")
                        continue  # 跳过时间重置
                    
                    # 重置轨迹开始时间
                    self.start_time = rospy.Time.now()
                    
            except Exception as e:
                rospy.logerr(f"输入错误: {str(e)}")
                break

    def generate_trajectory(self):
        """生成轨迹数据"""
        with self.lock:
            traj_type = self.trajectory_type
            t = (rospy.Time.now() - self.start_time).to_sec()

        if traj_type == "waiting":
            # 初始等待状态不发布指令
            return False
        elif traj_type == "circle":
            # 修正圆形轨迹公式 (半径2米，周期8秒)
            radius = 2.0
            omega = 2 / radius
            
            self.cmd_msg.position.x = radius * math.cos(omega * t)
            self.cmd_msg.position.y = radius * math.sin(omega * t)
            self.cmd_msg.position.z = 3
            
            self.cmd_msg.velocity.x = -radius * omega * math.sin(omega * t)
            self.cmd_msg.velocity.y = radius * omega * math.cos(omega * t)
            self.cmd_msg.velocity.z = 0.0
            
            self.cmd_msg.yaw = 0
            return True
            
        elif traj_type == "line":
            # 直线往复运动 (x轴方向)
            amplitude = 3.0
            period = 6.0
            
            self.cmd_msg.position.x = amplitude * math.sin(2 * math.pi / period * t)
            self.cmd_msg.position.y = 0.0
            self.cmd_msg.position.z = 3
            
            self.cmd_msg.velocity.x = amplitude * (2 * math.pi / period) * math.cos(2 * math.pi / period * t)
            self.cmd_msg.velocity.y = 0.0
            self.cmd_msg.velocity.z = 0.0
            
            self.cmd_msg.yaw = 0.0
            return True
            
        elif traj_type == "hover_origin":
            # 原点悬停 (0,0,3)
            self.cmd_msg.position.x = 0.0
            self.cmd_msg.position.y = 0.0
            self.cmd_msg.position.z = 3
            self.cmd_msg.velocity = Vector3(0,0,0)
            self.cmd_msg.yaw = 0.0
            return True
            
        elif traj_type == "hover_corner":
            # 角落悬停 (1,1,3)
            self.cmd_msg.position.x = 1.0
            self.cmd_msg.position.y = 1.0
            self.cmd_msg.position.z = 3
            self.cmd_msg.velocity = Vector3(0,0,0)
            self.cmd_msg.yaw = 0.0
            return True

    def run(self):
        while not rospy.is_shutdown():
            # 生成轨迹数据
            if self.generate_trajectory():
                # 更新header时间戳
                self.cmd_msg.header = Header(
                    stamp=rospy.Time.now(),
                    frame_id="world"
                )
                
                # 发布消息
                self.cmd_pub.publish(self.cmd_msg)
            
            # 维持发布频率
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TrajectoryPublisher()
        print("轨迹发布节点已启动，等待初始指令...")
        publisher.run()
    except rospy.ROSInterruptException:
        pass
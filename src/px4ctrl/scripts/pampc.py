#!/usr/bin/env python3
import rospy
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget, State
import numpy as np
import casadi as ca
import time  # 用于计时
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os  # 新增：用于文件路径处理
from datetime import datetime  # 新增：用于生成带时间戳的文件名
from scipy.spatial.transform import Rotation as R 

sys.path.insert(0, "/home/ris/SUPER_ws/src/px4ctrl/scripts")
from quadrotor_animation import QuadrotorAnimator

# 全局变量
state = [0, 0, 1,  # x, y, z
         0, 0, 0,  # vx, vy, vz
         1, 0, 0, 0  # qw, qx, qy, qz
         ]

ref_path = []
real_path = []
cmd_path = []
u_path = []
angular_path = []
time_path = []
mpc_ctrl = None  # 声明全局变量


class MPCController:
    def __init__(self):
        # 系统参数
        self.dt = 0.05  # 时间步长
        self.N = 10  # 预测时域
        # self.Q = np.diag([100, 100, 140, 10, 10, 20, 300, 300, 300])  # 状态权重
        self.Q = np.diag([200, 200, 80, 10, 10, 20, 2000, 2000, 2000])  # 状态权重
        self.R = np.diag([0.1, 1.0, 1.0, 1.0])  # 控制输入权重

        self.odom_init = False
        self.time_init = False
        self.start_time = None  # 存储开始时间

        # 定义系统模型 (三阶积分模型)
        self.setup_model()

    def setup_model(self):
        # 状态变量 [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        x = ca.MX.sym('x', 10)
        # 控制输入 [f/m, w_x, w_y, w_z] 
        u = ca.MX.sym('u', 4)

        # 动力学方程 (三阶积分模型)
        # a = f/m * R_3 - g * e_3
        xdot = ca.vertcat(
            x[3],           # dx/dt = vx
            x[4],           # dy/dt = vy
            x[5],           # dz/dt = vz
            2 * (x[6]*x[8] + x[7]*x[9]) * u[0],
            2 * (x[8]*x[9] - x[6]*x[7]) * u[0],
            (1 - 2 * x[7]**2 - 2 * x[8]**2) * u[0] - 9.81,
            0.5 * (- u[1] *x[7] - u[2] * x[8] - u[3] * x[9]),
            0.5 * (  u[1] *x[6] + u[3] * x[8] - u[2] * x[9]),
            0.5 * (  u[2] *x[6] - u[3] * x[7] + u[1] * x[9]),
            0.5 * (  u[3] *x[6] + u[2] * x[7] - u[1] * x[8]),
        )

        # 创建函数对象
        self.f = ca.Function('f', [x, u], [xdot], ['x', 'u'], ['xdot'])

    def solve_mpc(self, x0, ref_trajectory):

        global time_path

        opti = ca.Opti()
        X = opti.variable(10, self.N + 1)  # 状态序列
        U = opti.variable(4 , self.N)  # 控制序列

        # 初始条件约束
        opti.subject_to(X[:, 0] == x0)

        # 动力学约束
        for k in range(self.N):
            x_next = X[:, k] + self.f(X[:, k], U[:, k]) * self.dt
            opti.subject_to(X[:, k + 1] == x_next)

        # 成本函数
        cost = 0
        for k in range(self.N):
            # 跟踪误差成本
            error_pos = X[:6,k] - ref_trajectory[:6,k]
            cost += error_pos.T @ self.Q[:6,:6] @ error_pos

            error_q = ca.vertcat(
                ref_trajectory[7,k] * X[6,k] - ref_trajectory[6,k] * X[7,k] + ref_trajectory[9,k] * X[8,k]  - ref_trajectory[8,k] * X[9,k],
                ref_trajectory[8,k] * X[6,k] - ref_trajectory[9,k] * X[7,k] - ref_trajectory[6,k] * X[8,k]  + ref_trajectory[7,k] * X[9,k],
                ref_trajectory[9,k] * X[6,k] + ref_trajectory[8,k] * X[7,k] - ref_trajectory[7,k] * X[8,k]  - ref_trajectory[6,k] * X[9,k])

            cost += error_q.T @ self.Q[6:,6:] @ error_q

            # 控制输入成本
            cost += U[:, k].T @ self.R @ U[:, k]

        # 终端成本 (加大权重)
        error_terminal = X[:6, -1] - ref_trajectory[:6, -1]
        cost += 100* error_terminal.T @ error_terminal

        # max_theta = np.pi/6
        # opti.subject_to(opti.bounded(0, 1 - 2 * X[7,:]**2 - 2 * X[8,:]**2 - np.cos(max_theta), np.inf))

        # 边界约束
        max_thrust = 13.0
        opti.subject_to(opti.bounded(  9.81 , U[0,:], max_thrust)) # must be added

        max_w = 0.3
        opti.subject_to(opti.bounded(-max_w, U[1,:], max_w))
        opti.subject_to(opti.bounded(-max_w, U[2,:], max_w))
        opti.subject_to(opti.bounded(-max_w, U[3,:], max_w))

        # max_vel = 1.4
        # opti.subject_to(opti.bounded(-max_vel, X[3,:], max_vel))
        # opti.subject_to(opti.bounded(-max_vel, X[4,:], max_vel))
        # opti.subject_to(opti.bounded(-max_vel, X[5,:], max_vel))

        opti.minimize(cost)

        # 求解器设置
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        opti.solver('ipopt', opts)

        try:
            t1 = time.time()
            sol = opti.solve()
            print("求解成功!")
            time_path.append([time.time() - t1 , time.time() - mpc_ctrl.start_time,])
            return sol.value(U[:, 0]), sol.value(X[:, 1])
        except:
            print("求解失败!")
            return None, None

    def reference_trajectory(self, t):
        """生成参考轨迹 [sin(t), cos(t), 0.1t]"""
        ref = np.zeros((10, self.N))
        vel_xy = 2.0
        vel_z = 0.1
        for k in range(self.N):
            tmp = t + k * self.dt
            ref[0, k] =  vel_xy *  np.sin(tmp)
            ref[1, k] =  vel_xy * (np.cos(tmp) - 1.0)
            ref[2, k] =  vel_z  * (tmp) + 1.0

            ref[3, k] =  vel_xy * np.cos(tmp)
            ref[4, k] = -vel_xy * np.sin(tmp)
            ref[5, k] =  vel_z

            ax = -vel_xy * np.sin(tmp)
            ay = -vel_xy * np.cos(tmp)
            az = 0.0
            zb = np.array([ax, ay, az + 9.81])
            zb = zb / np.linalg.norm(zb)
            xc = np.array([1,0,0])

            yb = np.cross(zb, xc)
            yb = yb / np.linalg.norm(yb)

            xb = np.cross(yb, zb)
            xb = xb / np.linalg.norm(xb)

            rotation_matrix = np.column_stack((xb, yb, zb))
            rot = R.from_matrix(rotation_matrix)
            quaternion = rot.as_quat()
            ref[6,k] = quaternion[3]
            ref[7,k] = quaternion[0]
            ref[8,k] = quaternion[1]
            ref[9,k] = quaternion[2]

        return ref


def odom_callback(odom_msg: Odometry):
    global state, mpc_ctrl  # 声明使用全局变量

    if not mpc_ctrl.time_init:
        return
    state[0] = odom_msg.pose.pose.position.x
    state[1] = odom_msg.pose.pose.position.y
    state[2] = odom_msg.pose.pose.position.z
    state[3] = odom_msg.twist.twist.linear.x
    state[4] = odom_msg.twist.twist.linear.y
    state[5] = odom_msg.twist.twist.linear.z
    state[6] = odom_msg.pose.pose.orientation.w
    state[7] = odom_msg.pose.pose.orientation.x
    state[8] = odom_msg.pose.pose.orientation.y
    state[9] = odom_msg.pose.pose.orientation.z

    angular_path.append([odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z, time.time() - mpc_ctrl.start_time])

    mpc_ctrl.odom_init = True


def state_callback(msg):
    global mpc_ctrl
    if msg.mode == "OFFBOARD" and not mpc_ctrl.time_init:
        mpc_ctrl.start_time = time.time()
        mpc_ctrl.time_init = True
        print("OFFBOARD")

def timer_callback(event):
    global state, ref_path, real_path, cmd_path, u_path, mpc_ctrl

    if not mpc_ctrl.time_init or not mpc_ctrl.odom_init:
        msg = AttitudeTarget()
        msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        msg.thrust = 0.701
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        ctrl_attitude_pub_.publish(msg)
        return

    current_time = time.time() - mpc_ctrl.start_time
    ref = mpc_ctrl.reference_trajectory(current_time)
    u, pred = mpc_ctrl.solve_mpc(state, ref)

    if u is None:  # 修改这里
        return
    
    ref_path.append([ref[0, 0], ref[1, 0], ref[2, 0], ref[3, 0], ref[4, 0], ref[5, 0], ref[6, 0], ref[7, 0], ref[8, 0], ref[9, 0], current_time])
    real_path.append([state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9],current_time])
    cmd_path.append([pred[0], pred[1], pred[2], pred[3], pred[4], pred[5], pred[6], pred[7], pred[8], pred[9], current_time])  

    # 增加记录pred的代码    
    u_path.append([u[0], u[1], u[2], u[3], current_time])

    msg = AttitudeTarget()
    msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
    msg.thrust = u[0] * 0.706991 / 9.81
    msg.body_rate.x = u[1]
    msg.body_rate.y = u[2]
    msg.body_rate.z = u[3]
    ctrl_attitude_pub_.publish(msg)

def plot_and_save_results():
    """绘制并保存参考路径和实际路径的位置-时间、速度-时间曲线图"""
    global ref_path, real_path, cmd_path, u_path, angular_path,time_path
    
    if not ref_path or not real_path:
        print("没有足够的数据来绘制图表")
        return

    # 转换为numpy数组以便处理
    ref_array = np.array(ref_path)
    real_array = np.array(real_path)
    cmd_array = np.array(cmd_path)
    u_array = np.array(u_path)
    angular_array = np.array(angular_path)
    time_array = np.array(time_path)

    # 创建保存图片的目录（如果不存在）
    save_dir = "/home/ris/SUPER_ws/src/px4ctrl/figure/"
    os.makedirs(save_dir, exist_ok=True)

    # 创建画布
    plt.rcParams["font.family"] = ["SimHei", "WenQuanYi Micro Hei", "Heiti TC"]
    plt.rcParams["axes.unicode_minus"] = False
    fig = plt.figure(figsize=(10, 12))

    # 绘制位置-时间曲线
    ax = fig.add_subplot(411, projection='3d')
    ax.plot(ref_array[:, 0], ref_array[:, 1], ref_array[:, 2], 'r-', label='参考轨迹')
    # ax.plot(cmd_array[:, 0], cmd_array[:, 1], cmd_array[:, 2], 'g-', label='cmd轨迹')
    ax.plot(real_array[:, 0], real_array[:, 1], real_array[:, 2], 'b-', label='实际轨迹')
    ax.set_title('3D轨迹对比')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(ref_array[:, -1], ref_array[:, 0], 'r-', label='Ref x')
    plt.plot(ref_array[:, -1], ref_array[:, 1], 'g-', label='Ref y')
    plt.plot(ref_array[:, -1], ref_array[:, 2], 'b-', label='Ref z')
    plt.plot(real_array[:, -1], real_array[:, 0], 'r--', label='Real x')
    plt.plot(real_array[:, -1], real_array[:, 1], 'g--', label='Real y')
    plt.plot(real_array[:, -1], real_array[:, 2], 'b--', label='Real z')
    # 计算三维位置误差
    pos_error = np.sqrt((real_array[:, 0] - ref_array[:, 0])**2 + 
                        (real_array[:, 1] - ref_array[:, 1])**2 + 
                        (real_array[:, 2] - ref_array[:, 2])**2)
    # 计算位置误差的统计数据
    mean_error = np.mean(pos_error)
    std_error = np.std(pos_error)
    max_error = np.max(pos_error)
    # 在图表上显示统计数据
    stats_text = (f'位置误差统计:\n'
                f'平均误差 = {mean_error:.3f} m\n'
                f'标准差 = {std_error:.3f} m\n'
                f'最大误差 = {max_error:.3f} m')
    plt.text(0.05, 0.95, stats_text, transform=plt.gca().transAxes,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Position vs Time')
    plt.legend()
    plt.grid(True)

    # 绘制速度-时间曲线
    plt.subplot(4, 1, 3)
    plt.plot(ref_array[:, -1], ref_array[:, 3], 'r-', label='Ref vx')
    plt.plot(ref_array[:, -1], ref_array[:, 4], 'g-', label='Ref vy')
    plt.plot(ref_array[:, -1], ref_array[:, 5], 'b-', label='Ref vz')
    plt.plot(real_array[:, -1], real_array[:, 3], 'r--', label='Real vx')
    plt.plot(real_array[:, -1], real_array[:, 4], 'g--', label='Real vy')
    plt.plot(real_array[:, -1], real_array[:, 5], 'b--', label='Real vz')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity vs Time')
    plt.legend()
    plt.grid(True)

    # 绘制计算时间-时间曲线
    plt.subplot(4, 1, 4)
    plt.plot(time_array[:, -1], time_array[:, 0], 'r-', label='caculate time')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    save_path = os.path.join(save_dir, f"pampc_status.png")
    plt.savefig(save_path)
    plt.clf()
    plt.cla() 

    plt.subplot(4, 1, 1)
    plt.plot(u_array[:, 4], u_array[:, 0], 'r*', label='u[0]')
    plt.xlabel('Time (s)')
    plt.title('input 推力')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(u_array[:, 4], u_array[:, 1], 'r-', label='cmd wx')
    plt.plot(angular_array[:, 3], angular_array[:, 0], 'r--', label='real wx')
    plt.xlabel('Time (s)')
    plt.title('角速度 wx')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(u_array[:, 4], u_array[:, 2], 'g-', label='cmd wy')
    plt.plot(angular_array[:, 3], angular_array[:, 1], 'g--', label='real wy')
    plt.xlabel('Time (s)')
    plt.title('角速度 wy')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(u_array[:, 4], u_array[:, 3], 'b-', label='cmd wz')
    plt.plot(angular_array[:, 3], angular_array[:, 2], 'b--', label='real wz')
    plt.xlabel('Time (s)')
    plt.title('角速度 wz')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    save_path = os.path.join(save_dir, f"pampc_input.png")
    plt.savefig(save_path)
    plt.clf()
    plt.cla() 

    times = ref_array[:, -1]
    ref_q = ref_array[:, [7,8,9,6]]
    ref_r = R.from_quat(ref_q)
    ref_angle = ref_r.as_euler('xyz', degrees=True)

    real_q = real_array[:, [7,8,9,6]]
    real_r = R.from_quat(real_q)
    real_angle = real_r.as_euler('xyz', degrees=True)

    cmd_q = cmd_array[:, [7,8,9,6]]
    cmd_r = R.from_quat(cmd_q)
    cmd_angle = cmd_r.as_euler('xyz', degrees=True)

    # 滚转角（Roll）
    plt.subplot(3, 1, 1)
    plt.plot(times, ref_angle[:, 0], 'r-',label='Ref roll')
    plt.plot(times, cmd_angle[:, 0], 'g--',label='Cmd roll')
    plt.plot(times, real_angle[:, 0], 'b--',label='real roll')
    plt.title('滚转角(Roll)随时间变化')
    plt.ylabel('角度（度）')
    plt.grid(True)

    # 俯仰角（Pitch）
    plt.subplot(3, 1, 2)
    plt.plot(times, ref_angle[:, 1], 'r-',label='Ref Pitch')
    plt.plot(times, cmd_angle[:, 1], 'g--',label='Cmd Pitch')
    plt.plot(times, real_angle[:, 1], 'b--',label='Real Pitch')
    plt.title('俯仰角(Pitch)随时间变化')
    plt.ylabel('角度（度）')
    plt.grid(True)

    # 偏航角（Yaw）
    plt.subplot(3, 1, 3)
    plt.plot(times, ref_angle[:, 2], 'r-',label='Ref Yaw')
    plt.plot(times, cmd_angle[:, 2], 'g--',label='Cmd Yaw')
    plt.plot(times, real_angle[:, 2], 'b--',label='Real Yaw')
    plt.title('偏航角(Yaw)随时间变化')
    plt.xlabel('时间(s)')
    plt.ylabel('角度（度）')
    plt.grid(True)

    plt.tight_layout()

    save_path = os.path.join(save_dir, f"pampc_attitude.png")
    plt.savefig(save_path)
    print(f"图表已保存至: {save_path}")

    # 初始化动画类
    animator = QuadrotorAnimator(
        history=real_array,
        quaternions=real_q
    )
    animator.draw_animation(save_path="/home/ris/SUPER_ws/src/px4ctrl/figure/pampc_animation.gif", fps=20)

    # 显示图片（可选）
    # plt.show()


if __name__ == '__main__':
    try:
        # 初始化 ROS 节点
        rospy.init_node('pampc_controller', anonymous=True)

        mpc_ctrl = MPCController()

        ctrl_attitude_pub_ = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

        rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback)
        rospy.Subscriber('/mavros/state', State, state_callback)

        timer = rospy.Timer(rospy.Duration(mpc_ctrl.dt), timer_callback)

        print("节点已启动，按 Ctrl+C 终止...")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        # 在ROS节点关闭后绘制并保存图表
        print("正在绘制并保存轨迹图表...")
        plot_and_save_results()
        print("程序已退出")
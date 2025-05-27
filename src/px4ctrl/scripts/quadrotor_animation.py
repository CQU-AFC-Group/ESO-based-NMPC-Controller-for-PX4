import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R

class QuadrotorAnimator:
    def __init__(self, history, quaternions):
        self.history = history          # 实际状态历史（包含位置、速度、四元数）
        self.quaternions = quaternions  # 四元数历史
        self.times = history[:, -1]     # 时间序列

        # 提取位置数据（假设 history 中前3列为x,y,z）
        self.actual_pos = history[:, :3]

        # 转换四元数为旋转矩阵
        self.R_record = self._quat_to_rotmat()

    def _quat_to_rotmat(self):
        """将四元数转换为旋转矩阵序列"""
        r = R.from_quat(self.quaternions)  # 假设四元数顺序为 [qx, qy, qz, qw]
        return r.as_matrix()  # 返回 (n, 3, 3) 旋转矩阵数组

    def draw_animation(self, save_path=None, fps=30):
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.rcParams['axes.unicode_minus'] = False
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        
        # 设置坐标轴范围（根据实际数据调整）
        ax.set_xlim([self.actual_pos[:,0].min()-1, self.actual_pos[:,0].max()+1])
        ax.set_ylim([self.actual_pos[:,1].min()-1, self.actual_pos[:,1].max()+1])
        ax.set_zlim([0, self.actual_pos[:,2].max()+1])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('四旋翼三维轨迹与姿态动画')
        
        # 初始化轨迹线（仅保留实际轨迹）
        actual_line, = ax.plot([], [], [], 'b-', label='实际轨迹', linewidth=2)
        
        # 初始化四旋翼位置点（球体）
        quad_point, = ax.plot([], [], [], 'go', markersize=10, label='四旋翼')
        
        # 初始化姿态箭头（x: 红色, y: 黄色, z: 绿色）
        arrow_x = ax.quiver([], [], [], [], [], [], color='r', label='X轴', length=0.5)
        arrow_y = ax.quiver([], [], [], [], [], [], color='y', label='Y轴', length=0.5)
        arrow_z = ax.quiver([], [], [], [], [], [], color='g', label='Z轴', length=0.5)
        
        ax.legend()

        def init():
            actual_line.set_data([], [])
            actual_line.set_3d_properties([])
            quad_point.set_data([], [])
            quad_point.set_3d_properties([])
            arrow_x.set_segments([])
            arrow_y.set_segments([])
            arrow_z.set_segments([])
            return actual_line, quad_point, arrow_x, arrow_y, arrow_z

        def update(frame):
            # 更新轨迹
            actual_x, actual_y, actual_z = self.actual_pos[:frame+1, 0], self.actual_pos[:frame+1, 1], self.actual_pos[:frame+1, 2]
            
            actual_line.set_data(actual_x, actual_y)
            actual_line.set_3d_properties(actual_z)
            
            # 更新四旋翼位置
            current_pos = self.actual_pos[frame]
            quad_point.set_data([current_pos[0]], [current_pos[1]])
            quad_point.set_3d_properties([current_pos[2]])
            
            # 获取当前旋转矩阵
            R = self.R_record[frame]
            
            # 更新姿态箭头（箭头方向为旋转矩阵的列向量）
            arrow_x.set_segments([[current_pos, current_pos + R[:, 0]]])  # X轴方向（旋转矩阵第一列）
            arrow_y.set_segments([[current_pos, current_pos + R[:, 1]]])  # Y轴方向（旋转矩阵第二列）
            arrow_z.set_segments([[current_pos, current_pos + R[:, 2]]])  # Z轴方向（旋转矩阵第三列）
            
            return actual_line, quad_point, arrow_x, arrow_y, arrow_z

        # 生成动画
        ani = animation.FuncAnimation(
            fig, update, frames=len(self.times), init_func=init, blit=True, interval=1000/fps
        )
        
        if save_path:
            ani.save(save_path, writer='pillow', fps=fps, dpi=150)
        
        plt.close(fig)  # 关闭窗口避免占用内存
        print(f"动画已保存至：{save_path}")

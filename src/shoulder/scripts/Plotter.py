#!/usr/bin/env python3

import sys
print("Python executable:", sys.executable)
print("Python path:", sys.path)

import matplotlib.pyplot as plt
import numpy as np
from wrench_interfaces.srv import WrenchData
import rclpy
from rclpy.node import Node
import pandas as pd
from matplotlib.gridspec import GridSpec
import os
import subprocess
import threading


# 绘图节点
class Plotter(Node):
    def __init__(self):
        super().__init__("Plotter")
        self.srv = self.create_service(
            WrenchData,
            "wrench_data_service",
            self.handle_wrench_data_request
        )

        self.get_logger().info("Plotter 服务已启动，等待请求...")
    
    # 显示图片的函数
    def display_image(self, image_path):
        try:
            # 根据操作系统选择合适的命令
            if sys.platform.startswith('darwin'):  # macOS
                subprocess.run(['open', image_path])
            elif sys.platform.startswith('win'):   # Windows
                os.startfile(image_path)  # Windows特有的方法
            elif sys.platform.startswith('linux'): # Linux
                subprocess.run(['xdg-open', image_path])
            else:
                self.get_logger().warn(f"不支持的操作系统平台: {sys.platform}，无法自动打开图片")
        except Exception as e:
            self.get_logger().error(f"打开图片失败: {e}")

    # 处理请求
    def handle_wrench_data_request(self, request, response):
        csv_path = request.csv_file_path
        self.get_logger().info(f"收到请求，读取数据文件: {csv_path}")
        data = pd.read_csv(csv_path)
        data_matrix = data.values
        data_matrix = data_matrix.transpose()
        self.data_matrix = data_matrix
        curve_png = self.plot_curve()

        # 使用线程异步显示图片，避免阻塞服务
        display_thread = threading.Thread(target=self.display_image, args=(curve_png,))
        display_thread.daemon = True  
        display_thread.start()

        response.success = True
        response.message = f"Curve Plot saved to {curve_png}"
        response.image_path = curve_png

        return response

    # 绘制运动曲线
    def plot_curve(self):
        fig = plt.figure(figsize=(12, 6))
        gs = GridSpec(2, 2, width_ratios=[6, 4], height_ratios=[1, 1], hspace=0.4, wspace=0.3)
        # 左侧3D图：跨2行，第1列
        ax3d = fig.add_subplot(gs[:, 0], projection='3d')
        # 右侧上2D图：第1行，第2列
        ax2d_top = fig.add_subplot(gs[0, 1])
        # 右侧下2D图：第2行，第2列
        ax2d_bottom = fig.add_subplot(gs[1, 1])

        # 提取数据
        time = self.data_matrix[0]
        x = self.data_matrix[1]
        y = self.data_matrix[2]
        z = self.data_matrix[3]
        tau = np.sqrt(self.data_matrix[4]**2 + self.data_matrix[5]**2 + self.data_matrix[6]**2)
        f = np.sqrt(self.data_matrix[7]**2 + self.data_matrix[8]**2 + self.data_matrix[9]**2)

        # 绘制3D轨迹图
        norm = plt.Normalize(z.min(), z.max())
        colors = plt.cm.viridis(norm(z)) 
        for i in range(len(x)-1):
            ax3d.plot(x[i:i+2], y[i:i+2], z[i:i+2], 
                     color=colors[i], linewidth=2, alpha=0.8)
        ax3d.set_title('3D End-Effector Trajectory')
        ax3d.set_xlabel('X (m)', fontsize=12, labelpad=10)
        ax3d.set_ylabel('Y (m)', fontsize=12, labelpad=10)
        ax3d.set_zlabel('Z (m)', fontsize=12, labelpad=10)
        ax3d.xaxis.pane.fill = False
        ax3d.yaxis.pane.fill = False
        ax3d.zaxis.pane.fill = False
        ax3d.xaxis.pane.set_edgecolor('gray')
        ax3d.yaxis.pane.set_edgecolor('gray')
        ax3d.zaxis.pane.set_edgecolor('gray')
        ax3d.xaxis.pane.set_alpha(0.1)
        ax3d.yaxis.pane.set_alpha(0.1)
        ax3d.zaxis.pane.set_alpha(0.1)
        ax3d.set_box_aspect([1, 1, 0.8])  # 调整z轴比例

        # 绘制关节力矩随时间变化图
        ax2d_top.plot(time, tau, label='Torque', color='r')
        ax2d_top.set_title('Joint Torque')
        ax2d_top.set_xlabel('Time (s)')
        ax2d_top.set_ylabel('Torque (Nm)')
        ax2d_top.legend(loc="upper right")
        ax2d_top.grid(alpha=0.3)

        # 绘制肌肉力随时间变化图
        ax2d_bottom.plot(time, f, label='Force', color='g')
        ax2d_bottom.set_title('Muscle Force')
        ax2d_bottom.set_xlabel('Time (s)')
        ax2d_bottom.set_ylabel('Force (N)')
        ax2d_bottom.legend(loc="upper right")
        ax2d_bottom.grid(alpha=0.3)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        png_path = os.path.join(script_dir, "curve.png")
        plt.savefig(png_path, dpi=300, bbox_inches='tight')
        plt.tight_layout()
        self.get_logger().info("绘图完成。")
        plt.close(fig)
        
        return png_path


def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
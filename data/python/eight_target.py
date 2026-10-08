import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import os

# 1. 论文级别样式设置
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.size'] = 10
mpl.rcParams['axes.labelsize'] = 11
mpl.rcParams['axes.linewidth'] = 0.8

# 路径设置
CSV_FILE_PATH = 'data_analysis/csv/20260428_eight_Data.csv'
OUTPUT_DIR = 'output'
os.makedirs(OUTPUT_DIR, exist_ok=True)

# 周期参数设定 (根据 omega = 0.2 计算)
# 1个周期约 31.5s，2个周期约 63s
SHOW_DURATION = 18.0

def process_3d_trajectory(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)

    # 定义列名
    col_time = '__time'
    col_actual_x = '/fmu/out/vehicle_odometry/position[0]'
    col_actual_y = '/fmu/out/vehicle_odometry/position[1]'
    col_actual_z = '/fmu/out/vehicle_odometry/position[2]'
    col_target_x = '/uav/target_position/x'
    col_target_y = '/uav/target_position/y'
    col_target_z = '/uav/target_position/z'

    # 提取轨迹开始的时间点（target_x 第一个非空值的行）
    trajectory_start_idx = df[col_target_x].first_valid_index()
    if trajectory_start_idx is None:
        print("Error: No target trajectory data found in CSV.")
        return
    
    start_time = df.loc[trajectory_start_idx, col_time]
    end_time = start_time + SHOW_DURATION

    # 根据时间范围过滤数据 (只保留 1-2 个周期)
    df_filtered = df[(df[col_time] >= start_time) & (df[col_time] <= end_time)].copy()

    # 提取并去除空值
    actual_data = df_filtered[[col_actual_x, col_actual_y, col_actual_z]].dropna().copy()
    target_data = df_filtered[[col_target_x, col_target_y, col_target_z]].dropna().copy()

    # 关键修复：NED 坐标系 Z 取反转为高度
    actual_data['height'] = -actual_data[col_actual_z]
    target_data['height'] = -target_data[col_target_z]

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制期望轨迹 (Reference)
    ax.plot(target_data[col_target_x], 
            target_data[col_target_y], 
            target_data['height'], 
            color='black', linewidth=1.5, label='Reference', zorder=5)

    # 绘制实际轨迹 (Actual) - 修复了 linestyle 错误
    ax.plot(actual_data[col_actual_x], 
            actual_data[col_actual_y], 
            actual_data['height'], 
            color='#1f77b4', linestyle='--', linewidth=1.2, alpha=0.8, label='Actual', zorder=4)

    # 坐标轴设置
    ax.set_xlabel('X (m)', labelpad=10)
    ax.set_ylabel('Y (m)', labelpad=10)
    ax.set_zlabel('Altitude (m)', labelpad=10)
    
    # 按照要求固定 Z 轴
    ax.set_zlim(0, 14)
    
    # 根据 8 字大小自动微调 XY 范围，确保居中
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)

    ax.set_title('Dynamic Trajectory Tracking', pad=20)
    ax.legend(loc='upper right', frameon=True, edgecolor='black')

    # 保持物理比例，防止 8 字变形
    ax.set_box_aspect((1, 1, 0.7)) 

    # 导出文件
    output_path = os.path.join(OUTPUT_DIR, '20260428_eight.png')
    plt.savefig(output_path, dpi=600, bbox_inches='tight')
    plt.savefig(output_path.replace('.png', '.pdf'), format='pdf', bbox_inches='tight')
    
    print(f"Success! 2-cycle trajectory saved to {OUTPUT_DIR}/")
    plt.show()

if __name__ == "__main__":
    process_3d_trajectory(CSV_FILE_PATH)
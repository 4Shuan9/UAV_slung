import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import os

mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.size'] = 14
mpl.rcParams['axes.labelsize'] = 14
mpl.rcParams['axes.linewidth'] = 0.8

CSV_FILE_PATH = 'data_analysis/csv/20260429_eight_Data.csv'
OUTPUT_DIR = 'output'
os.makedirs(OUTPUT_DIR, exist_ok=True)

SHOW_DURATION = 31.5

def process_3d_trajectory(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)

    col_time = '__time'
    col_actual_x = '/fmu/out/vehicle_odometry/position[0]'
    col_actual_y = '/fmu/out/vehicle_odometry/position[1]'
    col_actual_z = '/fmu/out/vehicle_odometry/position[2]'
    
    col_target_x = '/uav/target_position/x'
    col_target_y = '/uav/target_position/y'
    col_target_z = '/uav/target_position/z'
    
    col_payload_x = '/model/payload_ball/pose/pose/position/x'
    col_payload_y = '/model/payload_ball/pose/pose/position/y'
    col_payload_z = '/model/payload_ball/pose/pose/position/z'

    trajectory_start_idx = df[col_actual_x].first_valid_index()
    if trajectory_start_idx is None:
        print("Error: No trajectory data found in CSV.")
        return
    
    start_time = df.loc[trajectory_start_idx, col_time]
    end_time = start_time + SHOW_DURATION

    df_filtered = df[(df[col_time] >= start_time) & (df[col_time] <= end_time)].copy()

    actual_data = df_filtered[[col_time, col_actual_x, col_actual_y, col_actual_z]].dropna().copy()
    target_data = df_filtered[[col_time, col_target_x, col_target_y, col_target_z]].dropna().copy()
    payload_data = df_filtered[[col_time, col_payload_x, col_payload_y, col_payload_z]].dropna().copy()

    actual_data['height'] = -actual_data[col_actual_z]
    if not target_data.empty:
        target_data['height'] = -target_data[col_target_z]

    payload_data['plot_x'] = payload_data[col_payload_y] 
    payload_data['plot_y'] = payload_data[col_payload_x] 
    payload_data['height'] = payload_data[col_payload_z] - 1

    fig = plt.figure(figsize=(8, 7))

    ax = fig.add_subplot(111, projection='3d')
    ax.xaxis.pane.set_facecolor("#ffffff00")
    ax.yaxis.pane.set_facecolor("#ffffff00")
    ax.zaxis.pane.set_facecolor("#ffffff00")

    ax.xaxis._axinfo["grid"]['linestyle']
    ax.yaxis._axinfo["grid"]['linestyle']
    ax.zaxis._axinfo["grid"]['linestyle']

    ax.zaxis.set_major_locator(plt.MultipleLocator(1.0))

    if not target_data.empty:
        ax.plot(target_data[col_target_x], 
                target_data[col_target_y], 
                target_data['height'], 
                color='black', linewidth=1.5, label='Reference', zorder=5)

    ax.plot(actual_data[col_actual_x], 
            actual_data[col_actual_y], 
            actual_data['height'], 
            color='#1f77b4', linestyle='--', linewidth=1.2, alpha=0.8, label='UAV Actual', zorder=4)

    ax.plot(payload_data['plot_x'], 
            payload_data['plot_y'], 
            payload_data['height'], 
            color="#d62728", linestyle='-.', linewidth=1.2, alpha=0.8, label='Load Actual', zorder=3)

    ax.set_xlabel('X (m)', labelpad=10)
    ax.set_ylabel('Y (m)', labelpad=10)
    ax.set_zlabel('Altitude (m)', labelpad=10)
    
    # 坐标轴范围
    ax.set_zlim(8.0, 11.0)
    ax.set_xlim(-8.0, 8.0)
    ax.set_ylim(-5.0, 5.0)

    # ax.set_title('Dynamic Trajectory Tracking with Load', pad=20)
    ax.legend(loc='upper right', frameon=True, edgecolor='black')

    ax.set_box_aspect((1, 1, 0.7)) 

    output_path = os.path.join(OUTPUT_DIR, '20260429_eight_with_payload.png')
    plt.savefig(output_path, dpi=600, bbox_inches='tight')
    plt.savefig(output_path.replace('.png', '.pdf'), format='pdf', bbox_inches='tight')
    
    print(f"Success! 轨迹图已保存到 {OUTPUT_DIR}/")
    plt.show()

if __name__ == "__main__":
    process_3d_trajectory(CSV_FILE_PATH)
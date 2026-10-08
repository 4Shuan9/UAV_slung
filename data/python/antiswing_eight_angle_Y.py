import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os

mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['axes.facecolor'] = 'white'
mpl.rcParams['figure.facecolor'] = 'white'
mpl.rcParams['font.size'] = 14
mpl.rcParams['axes.labelsize'] = 14
mpl.rcParams['axes.titlesize'] = 14
mpl.rcParams['legend.fontsize'] = 14
mpl.rcParams['xtick.labelsize'] = 14
mpl.rcParams['ytick.labelsize'] = 14
mpl.rcParams['axes.linewidth'] = 0.8

CSV_FILE_PATH = 'data_analysis/csv/20260429_antiswing_eight_Data.csv'
OUTPUT_DIR = 'output'
OUTPUT_FILENAME = '20260429_antiswing_eight_angle_Y' # 修改了输出文件名
TIME_RANGE = (0.0, 60.0)

# 定义两个放大区域的时间范围
ZOOM_RANGE_1 = (40.0, 45.0)  # 原来的放大范围
ZOOM_RANGE_2 = (21.0, 26.0)  # 新增的放大范围

Y_LIM_EST = (-50.0, 50.0)

def process_estimation_plot(file_path):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    df = pd.read_csv(file_path)
    
    # 仅保留所需列
    col_time = '__time'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'
    col_cv_comp_y = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/y'

    truth_data = df[[col_time, col_truth_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_y]].dropna().copy()
    cv_comp_data = df[[col_time, col_cv_comp_y]].dropna().copy()

    t0 = min(truth_data[col_time].min(), cv_data[col_time].min(), cv_comp_data[col_time].min())
    truth_data['t'] = truth_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0
    cv_comp_data['t'] = cv_comp_data[col_time] - t0

    truth_data = truth_data[(truth_data['t'] >= TIME_RANGE[0]) & (truth_data['t'] <= TIME_RANGE[1])]
    cv_data = cv_data[(cv_data['t'] >= TIME_RANGE[0]) & (cv_data['t'] <= TIME_RANGE[1])]
    cv_comp_data = cv_comp_data[(cv_comp_data['t'] >= TIME_RANGE[0]) & (cv_comp_data['t'] <= TIME_RANGE[1])]

    # 修改为1x1的单图画布，并缩减高度以保持比例美观
    fig, ax = plt.subplots(1, 1, figsize=(7.16, 4.0))
    fig.subplots_adjust(left=0.12, right=0.95, top=0.90, bottom=0.12)

    def plot_sub(ax, truth_t, truth_val, cv_t, cv_val, comp_t, comp_val, axis_name, title):
        ax.plot(truth_t, truth_val, color='black', linewidth=1.5, label='Truth')
        ax.plot(cv_t, cv_val, color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw')
        ax.plot(comp_t, comp_val, color='#d62728', linestyle='-.', linewidth=1.5, label='Comp')
        ax.set_ylabel(f'Angle ${axis_name}$ (deg)')
        ax.set_xlabel('Time (s)')
        ax.set_title(title, pad=10)
        ax.set_ylim(Y_LIM_EST)
        ax.set_xlim(TIME_RANGE)
        ax.grid(True, linestyle='--', color='#e0e0e0', alpha=0.7)
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='black', fancybox=False, framealpha=1.0)

        # 提取画局部放大的函数以避免代码重复
        def add_zoom_inset(bounds, zoom_range):
            # bounds 格式: [x0, y0, width, height] 相对于父坐标系的比例
            axins = ax.inset_axes(bounds)
            axins.plot(truth_t, truth_val, color='black', linewidth=1.2)
            axins.plot(cv_t, cv_val, color='#1f77b4', linestyle='--', linewidth=1.0)
            axins.plot(comp_t, comp_val, color='#d62728', linestyle='-.', linewidth=1.2)
            
            axins.set_xlim(zoom_range[0], zoom_range[1])
            mask = (truth_t >= zoom_range[0]) & (truth_t <= zoom_range[1])
            if any(mask):
                zoom_y = truth_val[mask]
                y_min, y_max = zoom_y.min(), zoom_y.max()
                margin = (y_max - y_min) * 0.3 if y_max != y_min else 5.0
                axins.set_ylim(y_min - margin, y_max + margin)
            
            axins.set_xticklabels([])
            axins.set_yticklabels([])
            axins.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
            ax.indicate_inset_zoom(axins, edgecolor="black", linewidth=0.8, alpha=0.3)

        # 1. 绘制第一个放大窗口 (右下部)
        add_zoom_inset([0.65, 0.05, 0.33, 0.33], ZOOM_RANGE_1)
        
        # 2. 绘制第二个放大窗口 (中上部)
        add_zoom_inset([0.35, 0.60, 0.28, 0.28], ZOOM_RANGE_2)

    # 仅调用 Y 轴绘制
    plot_sub(ax, truth_data['t'], truth_data[col_truth_y], 
             cv_data['t'], cv_data[col_cv_y], 
             cv_comp_data['t'], cv_comp_data[col_cv_comp_y], 'Y',
             title='')

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.pdf'), format='pdf', dpi=600, bbox_inches='tight')
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.png'), format='png', dpi=600, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    process_estimation_plot(CSV_FILE_PATH)
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os

# --- 全局绘图配置 ---
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

# --- 参数设置 ---
CSV_FILE_PATH = 'data_analysis/csv/20260427_swing_Data.csv'
OUTPUT_DIR = 'output'
OUTPUT_FILENAME = '20260427_swing_Y_axis_combined'

# 估计图 (上图) 参数
TIME_RANGE_EST = (0.0, 2.25)
ZOOM_RANGE = (1.25, 1.45)
Y_LIM_EST = (-50.0, 50.0)

# 误差图 (下图) 参数
TIME_RANGE_ERR = (0.0, 60.0)
Y_LIM_ERR = (-5.0, 5.0)

def process_combined_plot(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)
    
    # 仅保留Y轴需要的列名定义
    col_time = '__time'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'
    col_cv_comp_y = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/y'

    # 数据提取与对齐
    truth_data = df[[col_time, col_truth_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_y]].dropna().copy()
    cv_comp_data = df[[col_time, col_cv_comp_y]].dropna().copy()

    t0 = min(truth_data[col_time].min(), cv_data[col_time].min(), cv_comp_data[col_time].min())
    truth_data['t'] = truth_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0
    cv_comp_data['t'] = cv_comp_data[col_time] - t0

    # --- 估计图 (上图) 数据过滤 ---
    truth_est = truth_data[(truth_data['t'] >= TIME_RANGE_EST[0]) & (truth_data['t'] <= TIME_RANGE_EST[1])]
    cv_est = cv_data[(cv_data['t'] >= TIME_RANGE_EST[0]) & (cv_data['t'] <= TIME_RANGE_EST[1])]
    cv_comp_est = cv_comp_data[(cv_comp_data['t'] >= TIME_RANGE_EST[0]) & (cv_comp_data['t'] <= TIME_RANGE_EST[1])]

    # --- 误差图 (下图) 数据过滤 ---
    truth_err = truth_data[(truth_data['t'] >= TIME_RANGE_ERR[0]) & (truth_data['t'] <= TIME_RANGE_ERR[1])]
    cv_err = cv_data[(cv_data['t'] >= TIME_RANGE_ERR[0]) & (cv_data['t'] <= TIME_RANGE_ERR[1])]
    cv_comp_err = cv_comp_data[(cv_comp_data['t'] >= TIME_RANGE_ERR[0]) & (cv_comp_data['t'] <= TIME_RANGE_ERR[1])]

    # 计算误差 (插值对齐时间基准)
    error_cv_y = cv_err[col_cv_y].values - np.interp(cv_err['t'], truth_err['t'], truth_err[col_truth_y])
    error_cv_comp_y = cv_comp_err[col_cv_comp_y].values - np.interp(cv_comp_err['t'], truth_err['t'], truth_err[col_truth_y])

    # --- 创建 2x1 画布 ---
    fig, axs = plt.subplots(2, 1, figsize=(7.16, 7.16))
    fig.subplots_adjust(hspace=0.3, left=0.12, right=0.95, top=0.95, bottom=0.08)

    # ==========================================
    # (a) 绘制 Y 轴估计 vs 真值
    # ==========================================
    ax0 = axs[0]
    ax0.plot(truth_est['t'], truth_est[col_truth_y], color='black', linewidth=1.5, label='Truth')
    ax0.plot(cv_est['t'], cv_est[col_cv_y], color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw')
    ax0.plot(cv_comp_est['t'], cv_comp_est[col_cv_comp_y], color='#d62728', linestyle='-.', linewidth=1.5, label='Comp')
    ax0.set_ylabel('Angle $Y$ (°)')
    ax0.set_xlabel('Time (s)')
    ax0.set_title('(a) Estimation vs Truth (Y-Axis)', pad=10)
    ax0.set_ylim(Y_LIM_EST)
    ax0.set_xlim(TIME_RANGE_EST)
    ax0.grid(True, linestyle='--', color='#e0e0e0', alpha=0.7)
    ax0.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='black', fancybox=False, framealpha=1.0)

    # 绘制局部放大图
    axins = ax0.inset_axes([0.01, 0.64, 0.32, 0.32])
    axins.plot(truth_est['t'], truth_est[col_truth_y], color='black', linewidth=1.2)
    axins.plot(cv_est['t'], cv_est[col_cv_y], color='#1f77b4', linestyle='--', linewidth=1.0)
    axins.plot(cv_comp_est['t'], cv_comp_est[col_cv_comp_y], color='#d62728', linestyle='-.', linewidth=1.2)
    
    axins.set_xlim(ZOOM_RANGE[0], ZOOM_RANGE[1])
    mask = (truth_est['t'] >= ZOOM_RANGE[0]) & (truth_est['t'] <= ZOOM_RANGE[1])
    if any(mask):
        zoom_y = truth_est[col_truth_y][mask]
        y_min, y_max = zoom_y.min(), zoom_y.max()
        margin = (y_max - y_min) * 0.3 if y_max != y_min else 5.0
        axins.set_ylim(y_min - margin, y_max + margin)
    
    axins.set_xticklabels([])
    axins.set_yticklabels([])
    axins.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
    ax0.indicate_inset_zoom(axins, edgecolor="black", linewidth=0.8, alpha=0.3)

    # ==========================================
    # (b) 绘制 Y 轴误差
    # ==========================================
    ax1 = axs[1]
    ax1.plot(cv_err['t'], error_cv_y, color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw Error')
    ax1.plot(cv_comp_err['t'], error_cv_comp_y, color='#d62728', linestyle='-.', linewidth=1.5, label='Comp Error')
    
    # Raw Error 峰值辅助线与标签
    max_err_raw = np.max(error_cv_y)
    min_err_raw = np.min(error_cv_y)
    ax1.axhline(max_err_raw, color='#1f77b4', linestyle='--', linewidth=1.5, alpha=0.4)
    ax1.axhline(min_err_raw, color='#1f77b4', linestyle='--', linewidth=1.5, alpha=0.4)
    ax1.text(TIME_RANGE_ERR[0] + 0.5, max_err_raw + 0.1, f'{max_err_raw:.2f}', color='#1f77b4', va='bottom', ha='left', fontsize=12, fontweight='bold')
    ax1.text(TIME_RANGE_ERR[0] + 0.5, min_err_raw - 0.2, f'{min_err_raw:.2f}', color='#1f77b4', va='top', ha='left', fontsize=12, fontweight='bold')

    # Comp Error 峰值辅助线与标签
    max_err_comp = np.max(error_cv_comp_y)
    min_err_comp = np.min(error_cv_comp_y)
    ax1.axhline(max_err_comp, color='#d62728', linestyle=':', linewidth=1.5, alpha=0.5)
    ax1.axhline(min_err_comp, color='#d62728', linestyle=':', linewidth=1.5, alpha=0.5)
    ax1.text(TIME_RANGE_ERR[0] + 5.0, max_err_comp + 0.1, f'{max_err_comp:.2f}', color='#d62728', va='bottom', ha='left', fontsize=12, fontweight='bold')
    ax1.text(TIME_RANGE_ERR[0] + 5.0, min_err_comp - 0.21, f'{min_err_comp:.2f}', color='#d62728', va='top', ha='left', fontsize=12, fontweight='bold')

    # 轴属性设置
    ax1.set_ylabel('Error $Y$ (°)')
    ax1.set_xlabel('Time (s)')
    ax1.set_title('(b) Estimation Error (Y-Axis)', pad=10)
    ax1.set_ylim(Y_LIM_ERR)
    ax1.set_xlim(TIME_RANGE_ERR)
    ax1.grid(True, linestyle='--', color='#e0e0e0', alpha=0.7)
    ax1.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='black', fancybox=False, framealpha=1.0)

    # --- 统一格式与保存 ---
    fig.align_ylabels(axs)
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    # 按照要求仅输出一张PNG图片
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.png'), format='png', dpi=600, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    process_combined_plot(CSV_FILE_PATH)
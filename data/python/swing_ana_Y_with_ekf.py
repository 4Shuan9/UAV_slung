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
CSV_FILE_PATH = 'data_analysis/csv/20260625_swing_Data.csv'
OUTPUT_DIR = 'output'
OUTPUT_FILENAME = '20260625_swing_Y_axis_combined'

TIME_RANGE_EST = (0.0, 2.25)
ZOOM_RANGE = (0.80, 0.90)
Y_LIM_EST = (-50.0, 50.0)

TIME_RANGE_ERR = (0.0, 20.0)
Y_LIM_ERR = (-5.0, 5.0)

def process_combined_plot(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)
    
    # --- 列名定义 ---
    col_time = '__time'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'
    col_cv_Comp_y = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/y' 
    col_ekf_y = '/uav/ekf/swing_angle/world_frame/cartesian/vector/y' 
    col_eso_y = '/uav/eso/swing_angle/world_frame/cartesian/vector/y' 

    # --- 数据提取 ---
    truth_data = df[[col_time, col_truth_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_y]].dropna().copy()
    cv_Comp_data = df[[col_time, col_cv_Comp_y]].dropna().copy()
    ekf_data = df[[col_time, col_ekf_y]].dropna().copy()
    eso_data = df[[col_time, col_eso_y]].dropna().copy()

    # --- 时间基准对齐 ---
    t0 = min(truth_data[col_time].min(), cv_data[col_time].min(), 
             cv_Comp_data[col_time].min(), ekf_data[col_time].min(),
             eso_data[col_time].min())
             
    truth_data['t'] = truth_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0
    cv_Comp_data['t'] = cv_Comp_data[col_time] - t0
    ekf_data['t'] = ekf_data[col_time] - t0
    eso_data['t'] = eso_data[col_time] - t0

    # --- 估计图 (上图) 数据过滤 ---
    truth_est = truth_data[(truth_data['t'] >= TIME_RANGE_EST[0]) & (truth_data['t'] <= TIME_RANGE_EST[1])]
    cv_est = cv_data[(cv_data['t'] >= TIME_RANGE_EST[0]) & (cv_data['t'] <= TIME_RANGE_EST[1])]
    cv_Comp_est = cv_Comp_data[(cv_Comp_data['t'] >= TIME_RANGE_EST[0]) & (cv_Comp_data['t'] <= TIME_RANGE_EST[1])]
    ekf_est = ekf_data[(ekf_data['t'] >= TIME_RANGE_EST[0]) & (ekf_data['t'] <= TIME_RANGE_EST[1])]
    eso_est = eso_data[(eso_data['t'] >= TIME_RANGE_EST[0]) & (eso_data['t'] <= TIME_RANGE_EST[1])]

    # --- 误差图 (下图) 数据过滤 ---
    truth_err = truth_data[(truth_data['t'] >= TIME_RANGE_ERR[0]) & (truth_data['t'] <= TIME_RANGE_ERR[1])]
    cv_err = cv_data[(cv_data['t'] >= TIME_RANGE_ERR[0]) & (cv_data['t'] <= TIME_RANGE_ERR[1])]
    cv_Comp_err = cv_Comp_data[(cv_Comp_data['t'] >= TIME_RANGE_ERR[0]) & (cv_Comp_data['t'] <= TIME_RANGE_ERR[1])]
    ekf_err = ekf_data[(ekf_data['t'] >= TIME_RANGE_ERR[0]) & (ekf_data['t'] <= TIME_RANGE_ERR[1])]
    eso_err = eso_data[(eso_data['t'] >= TIME_RANGE_ERR[0]) & (eso_data['t'] <= TIME_RANGE_ERR[1])]

    # --- 计算误差 (插值对齐时间基准) ---
    error_cv_y = cv_err[col_cv_y].values - np.interp(cv_err['t'], truth_err['t'], truth_err[col_truth_y])
    error_cv_Comp_y = cv_Comp_err[col_cv_Comp_y].values - np.interp(cv_Comp_err['t'], truth_err['t'], truth_err[col_truth_y])
    error_ekf_y = ekf_err[col_ekf_y].values - np.interp(ekf_err['t'], truth_err['t'], truth_err[col_truth_y])
    error_eso_y = eso_err[col_eso_y].values - np.interp(eso_err['t'], truth_err['t'], truth_err[col_truth_y])

    # ==========================================
    # 计算并输出 MAE 和 RMSE (终端打印)
    # ==========================================
    def calc_metrics(error_array):
        mae = np.mean(np.abs(error_array))
        rmse = np.sqrt(np.mean(error_array**2))
        return mae, rmse

    mae_raw, rmse_raw = calc_metrics(error_cv_y)
    mae_comp, rmse_comp = calc_metrics(error_cv_Comp_y)
    mae_ekf, rmse_ekf = calc_metrics(error_ekf_y)
    mae_eso, rmse_eso = calc_metrics(error_eso_y)

    print(f"\n{'='*40}")
    print(f"{'Method':<10} | {'MAE (°)':<12} | {'RMSE (°)':<12}")
    print(f"{'-'*40}")
    print(f"{'RAW':<10} | {mae_raw:<12.4f} | {rmse_raw:<12.4f}")
    print(f"{'Comp':<10} | {mae_comp:<12.4f} | {rmse_comp:<12.4f}")
    print(f"{'EKF':<10} | {mae_ekf:<12.4f} | {rmse_ekf:<12.4f}")
    print(f"{'ESO':<10} | {mae_eso:<12.4f} | {rmse_eso:<12.4f}")
    print(f"{'='*40}\n")

    # --- 创建画布 ---
    fig, axs = plt.subplots(2, 1, figsize=(7.16, 7.16))
    fig.subplots_adjust(hspace=0.3, left=0.12, right=0.95, top=0.95, bottom=0.08)

    # ==========================================
    # (a) 绘制 Y 轴估计 vs 真值
    # ==========================================
    ax0 = axs[0]
    ax0.plot(truth_est['t'], truth_est[col_truth_y], color='black', linewidth=1.5, label='Truth')
    ax0.plot(cv_est['t'], cv_est[col_cv_y], color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw')
    ax0.plot(cv_Comp_est['t'], cv_Comp_est[col_cv_Comp_y], color='#d62728', linestyle='-.', linewidth=1.5, label='Comp')
    ax0.plot(ekf_est['t'], ekf_est[col_ekf_y], color="#ff7e33", linestyle='-', linewidth=1.0, label='EKF')
    ax0.plot(eso_est['t'], eso_est[col_eso_y], color="#2ca02c", linestyle=':', linewidth=1.2, label='ESO')
    
    ax0.set_ylabel('Angle $Y$ (°)')
    ax0.set_xlabel('Time (s)')
    ax0.set_title('(a) Estimation vs Truth (Y-Axis)', pad=10)
    ax0.set_ylim(Y_LIM_EST)
    ax0.set_xlim(TIME_RANGE_EST)
    ax0.grid(True, linestyle='--', color='#e0e0e0', alpha=0.7)
    ax0.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='black', fancybox=False, framealpha=1.0)

    # 绘制局部放大图
    axins = ax0.inset_axes([0.15, 0.05, 0.4, 0.4])
    axins.plot(truth_est['t'], truth_est[col_truth_y], color='black', linewidth=1.2)
    axins.plot(cv_est['t'], cv_est[col_cv_y], color='#1f77b4', linestyle='--', linewidth=1.0)
    axins.plot(cv_Comp_est['t'], cv_Comp_est[col_cv_Comp_y], color='#d62728', linestyle='-.', linewidth=1.2)
    axins.plot(ekf_est['t'], ekf_est[col_ekf_y], color='#ff7e33', linestyle='-', linewidth=1.0)
    axins.plot(eso_est['t'], eso_est[col_eso_y], color='#2ca02c', linestyle=':', linewidth=1.0)
    
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
    ax1.plot(cv_Comp_err['t'], error_cv_Comp_y, color='#d62728', linestyle='-.', linewidth=1.5, label='Comp Error')
    ax1.plot(ekf_err['t'], error_ekf_y, color='#ff7e33', linestyle='-', linewidth=1.0, label='EKF Error')
    ax1.plot(eso_err['t'], error_eso_y, color='#2ca02c', linestyle=':', linewidth=1.2, label='ESO Error')
    
    # 辅助线与标签
    def add_error_labels(ax, error_array, color, linestyle, time_offset):
        max_err = np.max(error_array)
        min_err = np.min(error_array)
        ax.axhline(max_err, color=color, linestyle=linestyle, linewidth=1.5, alpha=0.4)
        ax.axhline(min_err, color=color, linestyle=linestyle, linewidth=1.5, alpha=0.4)
        ax.text(TIME_RANGE_ERR[0] + time_offset, max_err + 0.1, f'{max_err:.2f}', color=color, va='bottom', ha='left', fontsize=12, fontweight='bold')
        ax.text(TIME_RANGE_ERR[0] + time_offset, min_err - 0.21, f'{min_err:.2f}', color=color, va='top', ha='left', fontsize=12, fontweight='bold')

    add_error_labels(ax1, error_cv_y, '#1f77b4', '--', 0.5)
    add_error_labels(ax1, error_cv_Comp_y, '#d62728', ':', 5.0)
    add_error_labels(ax1, error_ekf_y, '#ff7e33', '-', 9.5)
    add_error_labels(ax1, error_eso_y, '#2ca02c', ':', 14.0)

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
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.png'), format='png', dpi=600, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    process_combined_plot(CSV_FILE_PATH)
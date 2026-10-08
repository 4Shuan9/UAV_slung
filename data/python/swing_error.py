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
mpl.rcParams['font.size'] = 10
mpl.rcParams['axes.labelsize'] = 11
mpl.rcParams['axes.titlesize'] = 11
mpl.rcParams['legend.fontsize'] = 10
mpl.rcParams['xtick.labelsize'] = 9
mpl.rcParams['ytick.labelsize'] = 9
mpl.rcParams['axes.linewidth'] = 0.8

# --- 参数设置 ---
CSV_FILE_PATH = 'data_analysis/csv/20260427_swing_Data.csv'
OUTPUT_DIR = 'output'
OUTPUT_FILENAME = '20260427_swing_error'
TIME_RANGE = (0.0, 60.0)
Y_LIM_ERR = (-5.0, 5.0)

def process_error_plot(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)
    
    # 列名定义
    col_time = '__time'
    col_truth_x = '/uav/truth/swing_angle/world_frame/cartesian/vector/x'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_cv_x = '/uav/cv/swing_angle/world_frame/cartesian/vector/x'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'
    col_cv_comp_x = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/x'
    col_cv_comp_y = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/y'

    # 数据提取与对齐
    truth_data = df[[col_time, col_truth_x, col_truth_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_x, col_cv_y]].dropna().copy()
    cv_comp_data = df[[col_time, col_cv_comp_x, col_cv_comp_y]].dropna().copy()

    t0 = min(truth_data[col_time].min(), cv_data[col_time].min(), cv_comp_data[col_time].min())
    truth_data['t'] = truth_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0
    cv_comp_data['t'] = cv_comp_data[col_time] - t0

    # 时间范围过滤
    truth_data = truth_data[(truth_data['t'] >= TIME_RANGE[0]) & (truth_data['t'] <= TIME_RANGE[1])]
    cv_data = cv_data[(cv_data['t'] >= TIME_RANGE[0]) & (cv_data['t'] <= TIME_RANGE[1])]
    cv_comp_data = cv_comp_data[(cv_comp_data['t'] >= TIME_RANGE[0]) & (cv_comp_data['t'] <= TIME_RANGE[1])]

    # 计算误差 (插值对齐时间基准)
    error_cv_x = cv_data[col_cv_x].values - np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_x])
    error_cv_y = cv_data[col_cv_y].values - np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_y])
    error_cv_comp_x = cv_comp_data[col_cv_comp_x].values - np.interp(cv_comp_data['t'], truth_data['t'], truth_data[col_truth_x])
    error_cv_comp_y = cv_comp_data[col_cv_comp_y].values - np.interp(cv_comp_data['t'], truth_data['t'], truth_data[col_truth_y])

    # 绘图
    fig, axs = plt.subplots(2, 1, figsize=(7.16, 7.16))
    fig.subplots_adjust(hspace=0.3, left=0.12, right=0.95, top=0.95, bottom=0.08)

    def plot_sub_err(ax, cv_t, err_cv, comp_t, err_comp, axis_name, title):
        # 绘制误差曲线
        ax.plot(cv_t, err_cv, color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw Error')
        ax.plot(comp_t, err_comp, color='#d62728', linestyle='-.', linewidth=1.5, label='Comp Error')
        
        # --- Raw Error 峰值辅助线与标签 ---
        max_err_raw = np.max(err_cv)
        min_err_raw = np.min(err_cv)
        ax.axhline(max_err_raw, color='#1f77b4', linestyle='--', linewidth=1.5, alpha=0.4)
        ax.axhline(min_err_raw, color='#1f77b4', linestyle='--', linewidth=1.5, alpha=0.4)
        ax.text(TIME_RANGE[0] + 0.5, max_err_raw + 0.1, f'{max_err_raw:.2f}', color='#1f77b4', va='bottom', ha='left', fontsize=12, fontweight='bold')
        ax.text(TIME_RANGE[0] + 0.5, min_err_raw - 0.2, f'{min_err_raw:.2f}', color='#1f77b4', va='top', ha='left', fontsize=12, fontweight='bold')

        # --- Comp Error 峰值辅助线与标签 (新增) ---
        max_err_comp = np.max(err_comp)
        min_err_comp = np.min(err_comp)
        ax.axhline(max_err_comp, color='#d62728', linestyle=':', linewidth=1.5, alpha=0.5)
        ax.axhline(min_err_comp, color='#d62728', linestyle=':', linewidth=1.5, alpha=0.5)
        # 将 Comp 的文字标签向右偏移 (例如偏移 5s)，避免与 Raw 标签重叠
        ax.text(TIME_RANGE[0] + 5.0, max_err_comp + 0.1, f'{max_err_comp:.2f}', color='#d62728', va='bottom', ha='left', fontsize=12, fontweight='bold')
        ax.text(TIME_RANGE[0] + 5.0, min_err_comp -0.21, f'{min_err_comp:.2f}', color='#d62728', va='top', ha='left', fontsize=12, fontweight='bold')

        # 轴属性设置
        ax.set_ylabel(f'Error ${axis_name}$ (deg)')
        ax.set_xlabel('Time (s)')
        ax.set_title(title, pad=10)
        ax.set_ylim(Y_LIM_ERR)
        ax.set_xlim(TIME_RANGE)
        ax.grid(True, linestyle='--', color='#e0e0e0', alpha=0.7)
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='black', fancybox=False, framealpha=1.0)

    # 分别绘制 X 轴和 Y 轴误差
    plot_sub_err(axs[0], cv_data['t'], error_cv_x, cv_comp_data['t'], error_cv_comp_x, 'X', title='(a) Estimation Error (X-Axis)')
    plot_sub_err(axs[1], cv_data['t'], error_cv_y, cv_comp_data['t'], error_cv_comp_y, 'Y', title='(b) Estimation Error (Y-Axis)')

    fig.align_ylabels(axs)
    
    # 保存与展示
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.pdf'), format='pdf', dpi=600, bbox_inches='tight')
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.png'), format='png', dpi=600, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    process_error_plot(CSV_FILE_PATH)
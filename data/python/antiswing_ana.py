import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os

# ==========================================
# 1. 论文绘图全局配置 (严格符合 ICoIAS / IEEE 标准)
# ==========================================
# 设置全局字体为 Times New Roman
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'  # 数学公式符号字体与 Times New Roman 匹配
mpl.rcParams['axes.facecolor'] = 'white'   # 纯白背景
mpl.rcParams['figure.facecolor'] = 'white'
mpl.rcParams['font.size'] = 10             # 全局基准字号 (适合双栏排版)
mpl.rcParams['axes.labelsize'] = 11        # 坐标轴标签字号
mpl.rcParams['axes.titlesize'] = 11        # 标题字号
mpl.rcParams['legend.fontsize'] = 10       # 图例字号
mpl.rcParams['xtick.labelsize'] = 9        # 坐标轴刻度字号
mpl.rcParams['ytick.labelsize'] = 9
mpl.rcParams['axes.linewidth'] = 0.8       # 坐标轴边框粗细，学术标准

# ==========================================
# 2. 核心配置区
# ==========================================
CSV_FILE_PATH = 'data_analysis/csv/20260427_antiswing_Data.csv'
OUTPUT_DIR = 'output'                               # 保存路径
OUTPUT_FILENAME = '20260427_antiswing_Data'         # 保存的基础文件名
TIME_RANGE = (0.0, 3.0)                             # 时间轴显示范围 (秒)
Y_LIM_EST = (-1.0, 2.0)                             # 左侧估计值图的 Y 轴范围 (度)
Y_LIM_ERR = (-0.2, 0.5)                             # 右侧误差图的 Y 轴范围 (度)

# ==========================================
# 3. 数据处理与误差计算逻辑
# ==========================================
def process_drone_data(file_path):
    if not os.path.exists(file_path):
        print(f"Error: 文件 {file_path} 不存在。")
        return

    print(f"正在读取文件: {file_path} ...")
    df = pd.read_csv(file_path)

    # 定义列名
    col_time = '__time'
    col_truth_x = '/uav/truth/swing_angle/world_frame/cartesian/vector/x'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_cv_x = '/uav/cv/swing_angle/world_frame/cartesian/vector/x'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'
    col_cv_comp_x = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/x'
    col_cv_comp_y = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/y'

    # 提取并清理数据
    truth_data = df[[col_time, col_truth_x, col_truth_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_x, col_cv_y]].dropna().copy()
    cv_comp_data = df[[col_time, col_cv_comp_x, col_cv_comp_y]].dropna().copy()

    # 时间戳对齐并归零
    t0 = min(truth_data[col_time].min(), cv_data[col_time].min(), cv_comp_data[col_time].min())
    truth_data['t'] = truth_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0
    cv_comp_data['t'] = cv_comp_data[col_time] - t0

    # 截取设定时间范围内的独立数据
    truth_data = truth_data[(truth_data['t'] >= TIME_RANGE[0]) & (truth_data['t'] <= TIME_RANGE[1])]
    cv_data = cv_data[(cv_data['t'] >= TIME_RANGE[0]) & (cv_data['t'] <= TIME_RANGE[1])]
    cv_comp_data = cv_comp_data[(cv_comp_data['t'] >= TIME_RANGE[0]) & (cv_comp_data['t'] <= TIME_RANGE[1])]

    # 线性插值以对齐时间戳计算误差
    interp_truth_for_cv_x = np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_x])
    interp_truth_for_cv_y = np.interp(cv_data['t'], truth_data['t'], truth_data[col_truth_y])
    interp_truth_for_cv_comp_x = np.interp(cv_comp_data['t'], truth_data['t'], truth_data[col_truth_x])
    interp_truth_for_cv_comp_y = np.interp(cv_comp_data['t'], truth_data['t'], truth_data[col_truth_y])

    # 计算各轴误差
    error_cv_x = cv_data[col_cv_x].values - interp_truth_for_cv_x
    error_cv_y = cv_data[col_cv_y].values - interp_truth_for_cv_y
    error_cv_comp_x = cv_comp_data[col_cv_comp_x].values - interp_truth_for_cv_comp_x
    error_cv_comp_y = cv_comp_data[col_cv_comp_y].values - interp_truth_for_cv_comp_y

    # ==========================================
    # 4. 可视化绘图 (2x2 布局)
    # ==========================================
    # 7.16 英寸宽，完美适配 IEEE 双栏论文的跨栏大图
    fig, axs = plt.subplots(2, 2, figsize=(7.16, 6.0))
    # 调整间距，防止标题和上方刻度重叠
    fig.subplots_adjust(hspace=0.35, wspace=0.25, left=0.08, right=0.98, top=0.92, bottom=0.1)

    # 绘图函数封装：统一样式
    def plot_estimation(ax, truth_t, truth_val, cv_t, cv_val, comp_t, comp_val, axis_name, title):
        ax.plot(truth_t, truth_val, color='black', linewidth=1.5, label='Truth')
        ax.plot(cv_t, cv_val, color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw')
        ax.plot(comp_t, comp_val, color='#d62728', linestyle='-.', linewidth=1.5, label='Comp')
        ax.set_ylabel(f'Angle ${axis_name}$ (deg)')
        ax.set_xlabel('Time (s)')
        ax.set_title(title, pad=10) # 标题稍微拉开一点距离
        ax.set_ylim(Y_LIM_EST)
        ax.set_xlim(TIME_RANGE)

    def plot_error(ax, cv_t, err_cv, comp_t, err_comp, axis_name, title):
        ax.plot(cv_t, err_cv, color='#1f77b4', linestyle='--', linewidth=1.2, label='Raw Error')
        ax.plot(comp_t, err_comp, color='#d62728', linestyle='-.', linewidth=1.5, label='Comp Error')
        ax.set_ylabel(f'Error ${axis_name}$ (deg)')
        ax.set_xlabel('Time (s)')
        ax.set_title(title, pad=10)
        ax.set_ylim(Y_LIM_ERR)
        ax.set_xlim(TIME_RANGE)

    # [0, 0] 左上: (a) X轴估计
    plot_estimation(axs[0, 0], truth_data['t'], truth_data[col_truth_x], 
                    cv_data['t'], cv_data[col_cv_x], 
                    cv_comp_data['t'], cv_comp_data[col_cv_comp_x], 'X', 
                    title='(a) Estimation vs Truth (X-Axis)')

    # [0, 1] 右上: (b) X轴误差
    plot_error(axs[0, 1], cv_data['t'], error_cv_x, 
               cv_comp_data['t'], error_cv_comp_x, 'X',
               title='(b) Estimation Error (X-Axis)')

    # [1, 0] 左下: (c) Y轴估计
    plot_estimation(axs[1, 0], truth_data['t'], truth_data[col_truth_y], 
                    cv_data['t'], cv_data[col_cv_y], 
                    cv_comp_data['t'], cv_comp_data[col_cv_comp_y], 'Y',
                    title='(c) Estimation vs Truth (Y-Axis)')

    # [1, 1] 右下: (d) Y轴误差
    plot_error(axs[1, 1], cv_data['t'], error_cv_y, 
               cv_comp_data['t'], error_cv_comp_y, 'Y',
               title='(d) Estimation Error (Y-Axis)')

    # 统一设置图例、网格和边框
    for i in range(2):
        for j in range(2):
            ax = axs[i, j]
            # 浅灰色虚线网格
            ax.grid(True, linestyle='--', color='#e0e0e0', alpha=0.7)
            # 恢复标准的四周实线边框 (IEEE 标准要求框线完整)
            for spine in ax.spines.values():
                spine.set_visible(True)
                spine.set_linewidth(0.8)
            # 纯白背景图例放在右上角 (loc='upper right')
            ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='black', fancybox=False, framealpha=1.0)

    # 对齐所有 Y 轴标签，防止长短不一导致不整齐
    fig.align_ylabels(axs[:, 0])
    fig.align_ylabels(axs[:, 1])

    # ==========================================
    # 5. 创建输出目录并导出
    # ==========================================
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_pdf = os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.pdf')
    out_png = os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.png')

    # 导出高分辨率格式 (bbox_inches='tight' 可自动去除边缘多余白边)
    plt.savefig(out_pdf, format='pdf', dpi=600, bbox_inches='tight')
    plt.savefig(out_png, format='png', dpi=600, bbox_inches='tight')
    
    print(f"绘图完成！已保存为:")
    print(f" - {out_pdf}")
    print(f" - {out_png}")

    # 显示图表
    plt.show()

if __name__ == "__main__":
    process_drone_data(CSV_FILE_PATH)
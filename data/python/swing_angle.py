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
mpl.rcParams['font.size'] = 10
mpl.rcParams['axes.labelsize'] = 11
mpl.rcParams['axes.titlesize'] = 11
mpl.rcParams['legend.fontsize'] = 10
mpl.rcParams['xtick.labelsize'] = 9
mpl.rcParams['ytick.labelsize'] = 9
mpl.rcParams['axes.linewidth'] = 0.8

CSV_FILE_PATH = 'data_analysis/csv/20260427_swing_Data.csv'
OUTPUT_DIR = 'output'
OUTPUT_FILENAME = '20260427_swing_angle'
TIME_RANGE = (0.0, 2.5)
ZOOM_RANGE = (1.2, 1.4)
Y_LIM_EST = (-50.0, 50.0)

def process_estimation_plot(file_path):
    if not os.path.exists(file_path):
        return

    df = pd.read_csv(file_path)
    col_time = '__time'
    col_truth_x = '/uav/truth/swing_angle/world_frame/cartesian/vector/x'
    col_truth_y = '/uav/truth/swing_angle/world_frame/cartesian/vector/y'
    col_cv_x = '/uav/cv/swing_angle/world_frame/cartesian/vector/x'
    col_cv_y = '/uav/cv/swing_angle/world_frame/cartesian/vector/y'
    col_cv_comp_x = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/x'
    col_cv_comp_y = '/uav/cv_comp/swing_angle/world_frame/cartesian/vector/y'

    truth_data = df[[col_time, col_truth_x, col_truth_y]].dropna().copy()
    cv_data = df[[col_time, col_cv_x, col_cv_y]].dropna().copy()
    cv_comp_data = df[[col_time, col_cv_comp_x, col_cv_comp_y]].dropna().copy()

    t0 = min(truth_data[col_time].min(), cv_data[col_time].min(), cv_comp_data[col_time].min())
    truth_data['t'] = truth_data[col_time] - t0
    cv_data['t'] = cv_data[col_time] - t0
    cv_comp_data['t'] = cv_comp_data[col_time] - t0

    truth_data = truth_data[(truth_data['t'] >= TIME_RANGE[0]) & (truth_data['t'] <= TIME_RANGE[1])]
    cv_data = cv_data[(cv_data['t'] >= TIME_RANGE[0]) & (cv_data['t'] <= TIME_RANGE[1])]
    cv_comp_data = cv_comp_data[(cv_comp_data['t'] >= TIME_RANGE[0]) & (cv_comp_data['t'] <= TIME_RANGE[1])]

    fig, axs = plt.subplots(2, 1, figsize=(7.16, 7.16))
    fig.subplots_adjust(hspace=0.3, left=0.12, right=0.95, top=0.95, bottom=0.08)

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

        axins = ax.inset_axes([0.01, 0.64, 0.32, 0.32])
        axins.plot(truth_t, truth_val, color='black', linewidth=1.2)
        axins.plot(cv_t, cv_val, color='#1f77b4', linestyle='--', linewidth=1.0)
        axins.plot(comp_t, comp_val, color='#d62728', linestyle='-.', linewidth=1.2)
        
        axins.set_xlim(ZOOM_RANGE[0], ZOOM_RANGE[1])
        mask = (truth_t >= ZOOM_RANGE[0]) & (truth_t <= ZOOM_RANGE[1])
        if any(mask):
            zoom_y = truth_val[mask]
            y_min, y_max = zoom_y.min(), zoom_y.max()
            margin = (y_max - y_min) * 0.3 if y_max != y_min else 5.0
            axins.set_ylim(y_min - margin, y_max + margin)
        
        axins.set_xticklabels([])
        axins.set_yticklabels([])
        axins.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)
        ax.indicate_inset_zoom(axins, edgecolor="black", linewidth=0.8, alpha=0.3)

    plot_sub(axs[0], truth_data['t'], truth_data[col_truth_x], 
             cv_data['t'], cv_data[col_cv_x], 
             cv_comp_data['t'], cv_comp_data[col_cv_comp_x], 'X', 
             title='(a) Estimation vs Truth (X-Axis)')

    plot_sub(axs[1], truth_data['t'], truth_data[col_truth_y], 
             cv_data['t'], cv_data[col_cv_y], 
             cv_comp_data['t'], cv_comp_data[col_cv_comp_y], 'Y',
             title='(b) Estimation vs Truth (Y-Axis)')

    fig.align_ylabels(axs)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.pdf'), format='pdf', dpi=600, bbox_inches='tight')
    plt.savefig(os.path.join(OUTPUT_DIR, f'{OUTPUT_FILENAME}.png'), format='png', dpi=600, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    process_estimation_plot(CSV_FILE_PATH)
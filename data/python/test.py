import pandas as pd
import numpy as np

# 1. 读取 CSV 文件
df = pd.read_csv('data_analysis/csv/20260429_antiswing_eight_Data.csv')

# 2. 提取真值(truth)的摆角数据，并清理缺失值
df_truth = df[['__time', 
               '/uav/truth/swing_angle/world_frame/cartesian/vector/x', 
               '/uav/truth/swing_angle/world_frame/cartesian/vector/y']].dropna()

# 将时间戳转化为以秒为单位的相对时间
df_truth['time'] = (df_truth['__time'] - df_truth['__time'].iloc[0])

# 3. 剥离初始的未受控震荡/收敛阶段（通常在前25秒左右），提取动态机动阶段数据
df_maneuver = df_truth[df_truth['time'] > 40.0]

# 4. 计算动态机动期间，X轴和Y轴的最大绝对摆角
max_abs_x = df_maneuver['/uav/truth/swing_angle/world_frame/cartesian/vector/x'].abs().max()
max_abs_y = df_maneuver['/uav/truth/swing_angle/world_frame/cartesian/vector/y'].abs().max()

# 5. 取单轴中最极端的情况作为安全区间包络
max_swing_angle = max(max_abs_x, max_abs_y)

print(f"动态机动阶段 X 轴最大摆角: ±{max_abs_x:.2f}°")
print(f"动态机动阶段 Y 轴最大摆角: ±{max_abs_y:.2f}°")
print(f"综合最大摆角安全包络 (XX):   ±{max_swing_angle:.2f}°")
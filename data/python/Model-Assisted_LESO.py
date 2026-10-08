import numpy as np
import matplotlib.pyplot as plt

# --- 1. 参数设置 ---
g = 9.81     # 重力加速度 [m/s^2]
l = 1.0      # 吊挂绳长 [m]

# 【修改点1】推算并调整观测器带宽
# 视觉频率25Hz -> 采样角频率约 157 rad/s。带宽一般取采样频率的 1/10 左右。
# 考虑到0.04s的视觉延迟，设为 10.0 比较稳定。
w_o = 25.0   
b0 = -1.0/l  # 系统输入控制增益

# 根据文档进行LESO带宽法配置极点 (Model-Assisted LESO)
beta1 = 3 * w_o
beta2 = 3 * w_o**2 - (g / l)
beta3 = w_o**3

# --- 2. 仿真环境设置 (多采样率) ---
dt = 0.001         # 物理引擎仿真步长 (1000Hz)
T = 10.0           # 仿真总时间
t = np.arange(0, T, dt)
n = len(t)

# 传感器采样间隔设置
freq_vis = 25      # 视觉频率 25Hz
freq_imu = 200     # IMU频率 200Hz
step_vis = int((1.0 / freq_vis) / dt)  # 每 40 步更新一次视觉
step_imu = int((1.0 / freq_imu) / dt)  # 每 5 步更新一次 IMU

# --- 3. 状态变量初始化 ---
x = np.zeros((2, n))
x[0, 0] = 0.2  # 真实系统初始摆角 0.2 rad

z = np.zeros((3, n))  # 观测器状态初始为 0

# --- 4. 外部输入与扰动 ---
u_true = 2.0 * np.sin(1.5 * t)  # 真实无人机加速度
d = 0.5 * np.cos(3 * t) + 0.1 * np.random.randn(n)  # 真实环境扰动

f_true = np.zeros(n)

# 传感器缓存区 (零阶保持器)
y_meas = x[0, 0]   # 视觉测量缓存
u_meas = u_true[0] # IMU测量缓存

# --- 5. 仿真主循环 ---
for i in range(n-1):
    # 【A. 真实物理引擎 (1000Hz 连续运行)】
    theta_ddot = -(g/l) * np.sin(x[0, i]) - (u_true[i]/l) * np.cos(x[0, i]) + d[i]
    x[0, i+1] = x[0, i] + x[1, i] * dt
    x[1, i+1] = x[1, i] + theta_ddot * dt
    f_true[i] = -(g/l) * (np.sin(x[0, i]) - x[0, i]) - (u_true[i]/l) * (np.cos(x[0, i]) - 1.0) + d[i]
    
    # 【B. 传感器采样环节】
    # 视觉信号 25Hz 采样 (产生阶梯状信号)
    if i % step_vis == 0:
        y_meas = x[0, i] 
    
    # IMU信号 200Hz 采样
    if i % step_imu == 0:
        u_meas = u_true[i]
        
    # 【C. LESO 观测器算法 (使用离散采样信号更新)】
    e = y_meas - z[0, i]  # 观测误差现在使用的是25Hz的低频视觉数据
    
    z1_dot = z[1, i] + beta1 * e
    z2_dot = -(g/l) * z[0, i] + b0 * u_meas + z[2, i] + beta2 * e  # 使用200Hz的IMU数据
    z3_dot = beta3 * e
    
    # 欧拉法更新观测器状态
    z[0, i+1] = z[0, i] + z1_dot * dt
    z[1, i+1] = z[1, i] + z2_dot * dt
    z[2, i+1] = z[2, i] + z3_dot * dt

# 补全最后一点的真实扰动
f_true[-1] = -(g/l) * (np.sin(x[0, -1]) - x[0, -1]) - (u_true[-1]/l) * (np.cos(x[0, -1]) - 1.0) + d[-1]

# --- 6. 计算 RMSE (均方根误差) ---
# 截取后半段数据计算RMSE，避免初始收敛阶段（瞬态）的巨大误差影响稳态评价
eval_start = int(n * 0.2)  # 取 2.0s 之后的数据计算 RMSE
rmse_x1 = np.sqrt(np.mean((x[0, eval_start:] - z[0, eval_start:])**2))
rmse_x2 = np.sqrt(np.mean((x[1, eval_start:] - z[1, eval_start:])**2))
rmse_x3 = np.sqrt(np.mean((f_true[eval_start:] - z[2, eval_start:])**2))

# 终端输出
print("--- 稳态跟踪 RMSE (均方根误差) ---")
print(f"Angle (x1) RMSE:       {rmse_x1:.4f} rad")
print(f"Angular Vel (x2) RMSE: {rmse_x2:.4f} rad/s")
print(f"Disturbance (x3) RMSE: {rmse_x3:.4f}")

# --- 7. 绘图验证 ---
plt.figure(figsize=(10, 8), dpi=100)

# 生成包含 Omega 和 RMSE 信息的全局标题
title_str = (f"Model-Assisted LESO Verification ($\omega_o = {w_o}$ rad/s)\n"
             f"Vis: {freq_vis}Hz | IMU: {freq_imu}Hz | "
             f"RMSE $\\theta$: {rmse_x1:.4f} | RMSE $\\dot{{\\theta}}$: {rmse_x2:.4f} | RMSE $f$: {rmse_x3:.4f}")
plt.suptitle(title_str, fontsize=14, fontweight='bold')

plt.subplot(3, 1, 1)
plt.plot(t, x[0, :], label='True Angle $\\theta$', color='black')
plt.plot(t, z[0, :], '--', label='Estimated Angle $\\hat{x}_1$', color='red')
plt.ylabel('Angle (rad)')
plt.legend(loc='upper right')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t, x[1, :], label='True Angular Vel $\\dot{\\theta}$', color='black')
plt.plot(t, z[1, :], '--', label='Estimated Vel $\\hat{x}_2$', color='blue')
plt.ylabel('Angular Vel (rad/s)')
plt.legend(loc='upper right')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(t, f_true, label='True Lumped Disturbance $f(t)$', color='black', alpha=0.6)
plt.plot(t, z[2, :], '--', label='Estimated Disturbance $\\hat{x}_3$', color='green')
plt.ylabel('Disturbance $f(t)$')
plt.xlabel('Time (s)')
plt.legend(loc='upper right')
plt.grid(True)

plt.tight_layout(rect=[0, 0, 1, 0.95]) # 留出顶部标题空间
plt.show()
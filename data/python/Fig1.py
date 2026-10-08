import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

# 构建 3D 箭头类
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)

# 封装 3D 箭头绘制函数
def add_3d_arrow(ax, x, y, z, dx, dy, dz, color='k', label='', text_offset=(0.2, 0.2, 0.2), linestyle='-'):
    arrow = Arrow3D([x, x+dx], [y, y+dy], [z, z+dz], mutation_scale=15, 
                    lw=2, arrowstyle="-|>", color=color, linestyle=linestyle)
    ax.add_artist(arrow)
    if label:
        # 移除强制的 fontweight/style，交由 \boldsymbol 和全局设置处理
        ax.text(x+dx+text_offset[0], y+dy+text_offset[1], z+dz+text_offset[2], 
                label, fontsize=16, color=color)

# === 初始化图表环境（字体加粗核心设置） ===
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['font.weight'] = 'bold'       # 常规文本加粗
plt.rcParams['axes.labelweight'] = 'bold'  # 坐标轴标签加粗
plt.rcParams['mathtext.fontset'] = 'stix'  # STIX 字体库最接近 Times 的数学字体格式
os.makedirs('output', exist_ok=True)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.axis('off')

# 系统物理参数
L = 5
theta_x = np.radians(25)
theta_y = np.radians(35)

# 计算负载在世界坐标系下的位置
Z_val = L / np.sqrt(1 + np.tan(theta_x)**2 + np.tan(theta_y)**2)
X = Z_val * np.tan(theta_x)
Y = -Z_val * np.tan(theta_y)  
Z = -Z_val 

# 设置无人机倾斜姿态及旋转矩阵
pitch = np.radians(10)
roll = np.radians(10)

Rx = np.array([[1, 0, 0],
               [0, np.cos(roll), -np.sin(roll)],
               [0, np.sin(roll), np.cos(roll)]])
Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
               [0, 1, 0],
               [-np.sin(pitch), 0, np.cos(pitch)]])
R = Ry @ Rx

# 绘制倾斜的无人机机臂
arm = 2
r_d = arm * np.cos(np.pi/4)
arm_pts = np.array([[-r_d, -r_d, 0], [r_d, r_d, 0], [-r_d, r_d, 0], [r_d, -r_d, 0]])
rot_arm_pts = (R @ arm_pts.T).T

ax.plot([rot_arm_pts[0,0], rot_arm_pts[1,0]], [rot_arm_pts[0,1], rot_arm_pts[1,1]], [rot_arm_pts[0,2], rot_arm_pts[1,2]], 'k-', lw=3)
ax.plot([rot_arm_pts[2,0], rot_arm_pts[3,0]], [rot_arm_pts[2,1], rot_arm_pts[3,1]], [rot_arm_pts[2,2], rot_arm_pts[3,2]], 'k-', lw=3)

# 绘制倾斜的旋翼及电机轴
theta = np.linspace(0, 2*np.pi, 50)
for i in range(4):
    cx, cy, cz = rot_arm_pts[i]
    circle_pts = np.array([0.5*np.cos(theta), 0.5*np.sin(theta), np.zeros_like(theta)])
    rot_circle = (R @ circle_pts).T
    ax.plot(cx + rot_circle[:,0], cy + rot_circle[:,1], cz + rot_circle[:,2], 'k-', lw=1.5)
    
    axis_vec = R @ np.array([0, 0, 0.05])
    ax.plot([cx, cx + axis_vec[0]], [cy, cy + axis_vec[1]], [cz, cz + axis_vec[2]], 'k-', lw=5)

# 绘制世界坐标系 (W) 与机体 Z 轴 (B)
# 注意：使用 r'$\boldsymbol{...}$' 将数学公式和变量加粗
add_3d_arrow(ax, 0, 0, 0, 3, 0, 0, color='k', label=r'$\boldsymbol{X_W}$', text_offset=(0.1, 0, 0))
add_3d_arrow(ax, 0, 0, 0, 0, -6, 0, color='k', label=r'$\boldsymbol{Y_W}$', text_offset=(0, -0.1, 0))
add_3d_arrow(ax, 0, 0, 0, 0, 0, -5, color='k', label=r'$\boldsymbol{Z_W}$', text_offset=(0, 0, -0.2))
ax.text(-0.3, 0.3, 0.3, r'$\boldsymbol{O_W}$', fontsize=16)

Z_b_vec = R @ np.array([0, 0, -5]) 
add_3d_arrow(ax, 0, 0, 0, Z_b_vec[0], Z_b_vec[1], Z_b_vec[2], color='gray', label=r'$\boldsymbol{Z_B}$', text_offset=(0.1, 0, -0.2), linestyle='-.')

# 绘制吊绳与负载
ax.plot([0, X], [0, Y], [0, Z], color="black", lw=3)
ax.scatter([X], [Y], [Z], color='#d62728', s=200, depthshade=False, zorder=5)
ax.text(X+0.1, Y-0.3, Z, r'$\boldsymbol{m}$', fontsize=16)

# 绘制投影辅助线
ax.plot([X, X], [Y, 0], [Z, Z], 'k--', alpha=0.5, lw=1.2)
ax.plot([X, 0], [Y, Y], [Z, Z], 'k--', alpha=0.5, lw=1.2)
ax.plot([X, 0], [0, 0], [Z, Z], 'k--', alpha=0.5, lw=1.2)
ax.plot([0, 0], [Y, 0], [Z, Z], 'k--', alpha=0.5, lw=1.2)
ax.plot([0, 0], [0, 0], [0, Z], 'k--', alpha=0.5, lw=1.2)
ax.plot([X, 0], [Y, 0], [Z, Z], 'k--', alpha=0.5, lw=1.2)
ax.plot([0, X], [0, 0], [0, Z], '--', color='#d62728', lw=1.75)
ax.plot([0, 0], [0, Y], [0, Z], '--', color='#1f77b4', lw=1.75)

# 绘制摆角圆弧及标注
t = np.linspace(0, theta_x, 30)
arc_r = 2.1
ax.plot(arc_r*np.sin(t), np.zeros_like(t), -arc_r*np.cos(t), '#d62728', lw=1.5)
ax.text(arc_r*np.sin(theta_x/2) - 0.2, -0.15, -arc_r*np.cos(theta_x/2) -0.3, r'$\boldsymbol{\theta_x}$', color='#d62728', fontsize=16)

t2 = np.linspace(0, theta_y, 30)
arc_r2 = 1.25
ax.plot(np.zeros_like(t2), -arc_r2*np.sin(t2), -arc_r2*np.cos(t2), '#1f77b4', lw=1.5)
ax.text(-0.2, -arc_r2*np.sin(theta_y/2) - 0.2, -arc_r2*np.cos(theta_y/2) - 0.2, r'$\boldsymbol{\theta_y}$', color='#1f77b4', fontsize=16)

# 视角设置与保存
ax.view_init(elev=15, azim=-115)
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
plt.savefig('output/Fig1.png', dpi=600, transparent=False, facecolor='white', bbox_inches='tight', pad_inches=0)
plt.savefig('output/Fig1.pdf', transparent=False, facecolor='white', bbox_inches='tight', pad_inches=0)
plt.show()
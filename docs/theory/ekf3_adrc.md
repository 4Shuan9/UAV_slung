# 四旋翼无人机吊挂系统
# 数学模型 + EKF3 + ADRC

## 一、定义变量

| 符号 | 物理含义 |
|:---:|:---:|
| $m$ | 吊载质量（kg） |
| $l$ | 绳索长度（m） |
| $g$ | 重力加速度（m/s²） |
| $\theta$ | 吊载绳索相对竖直向下方向的有符号摆角（rad） |
| $\omega=\dot\theta$ | 摆角速度（rad/s） |
| $a$ | 无人机悬挂点沿研究平面的水平加速度（m/s²） |
| $\tau_d$ | 未建模力矩及外部扰动力矩（N·m） |
| $f=\tau_d/(ml^2)$ | 等效角加速度扰动（rad/s²） |
| $y_k$ | 在 $t_k$ 时刻测得的视觉摆角（rad） |
| $\hat x$ | EKF3 估计的扩张状态 |
| $P$ | 状态估计误差协方差矩阵 |
| $Q_c$ | 连续时间过程噪声强度矩阵 |
| $R_k$ | 第 $k$ 次视觉测量噪声方差 |
| $K_k$ | 第 $k$ 次视觉测量对应的 Kalman 增益 |

假设载荷为质点，绳索质量可忽略、长度恒定且始终张紧；先研究一个竖直摆动平面，并忽略基座竖直加速度和两摆动平面的耦合。

---

## 二、根据牛顿—欧拉方程建立非线性动力学方程

载荷绕悬挂点的转动惯量：

$$
\boxed{J=ml^2}
\tag{1}
$$

重力产生的恢复力矩：

$$
\boxed{\tau_g=-mgl\sin\theta}
\tag{2}
$$

无人机悬挂点水平加速所对应的惯性力矩：

$$
\boxed{\tau_a=-mal\cos\theta}
\tag{3}
$$

根据绕悬挂点的转动定律：

$$
J\ddot\theta=\tau_g+\tau_a+\tau_d
\tag{4}
$$

代入各项：

$$
ml^2\ddot\theta=-mgl\sin\theta-mal\cos\theta+\tau_d
\tag{5}
$$

将未知扰动定义为等效角加速度上的扰动：

$$
\boxed{
f(t)=\frac{\tau_d(t)}{J}=\frac{\tau_d(t)}{ml^2}
}
\tag{6}
$$

得：

$$
\boxed{
\ddot\theta
=-\frac{g}{l}\sin\theta
-\frac{a}{l}\cos\theta
+f(t)
}
\tag{7}
$$

---

## 三、数学重构：非线性三维扩张状态空间方程

定义状态变量：

$$
\boxed{
\begin{aligned}
x_1&=\theta,\\
x_2&=\dot\theta=\omega,\\
x_3&=f.
\end{aligned}}
\tag{8}
$$

组成三维扩张状态：

$$
\boxed{
x=\begin{bmatrix}x_1\\x_2\\x_3\end{bmatrix}
=\begin{bmatrix}\theta\\\omega\\f\end{bmatrix}
}
\tag{9}
$$

以基座水平加速度为系统的已知动力学输入：

$$
\boxed{u=a}
\tag{10}
$$

对三个状态求导，并将未知扰动变化表示为 $h(t)=\dot f(t)$：

$$
\boxed{
\begin{cases}
\dot x_1=x_2,\\[3pt]
\dot x_2=-\dfrac{g}{l}\sin x_1-\dfrac{u}{l}\cos x_1+x_3,\\[5pt]
\dot x_3=h(t).
\end{cases}}
\tag{11}
$$

为建立 EKF3，将不可精确预测的动力学误差与扰动变化近似为零均值过程噪声：

$$
\begin{cases}
\dot x_1=x_2,\\[3pt]
\dot x_2=-\dfrac{g}{l}\sin x_1-\dfrac{u}{l}\cos x_1+x_3+w_\omega,\\[5pt]
\dot x_3=w_f.
\end{cases}
\tag{12}
$$

因此，非线性随机状态空间模型写为：

$$
\boxed{\dot x=F(x,u)+Gw}
\tag{13}
$$

其中：

$$
\boxed{
F(x,u)=
\begin{bmatrix}
x_2\\[3pt]
-\dfrac{g}{l}\sin x_1-\dfrac{u}{l}\cos x_1+x_3\\[5pt]
0
\end{bmatrix}}
\tag{14}
$$

$$
\boxed{
G=\begin{bmatrix}0&0\\1&0\\0&1\end{bmatrix},\qquad
w=\begin{bmatrix}w_\omega\\w_f\end{bmatrix}}
\tag{15}
$$

设过程噪声**零均值、相互独立**，且具有连续白噪声强度：

$$
\mathbb E[w(t)]=0,\qquad
\mathbb E\!\left[w(t)w(s)^T\right]=Q_c\delta(t-s)
\tag{16}
$$

$$
\boxed{
Q_c=\begin{bmatrix}q_\omega&0\\0&q_f\end{bmatrix}}
\tag{17}
$$

视觉直接测量摆角 $\theta$ 值为真实摆角 + 测量误差，故测量方程为：

$$
\boxed{y_k=H x(t_k)+v_k}
\tag{18}
$$

$$
\boxed{H=\begin{bmatrix}1&0&0\end{bmatrix}}
\tag{19}
$$

$$
\mathbb E[v_k]=0,\qquad
\mathbb E[v_k^2]=R_k>0
\tag{20}
$$

至此已知**加速度输入 $u$、单摆角视觉输出 $y$、三维扩张状态 $x$** 的非线性状态估计问题。

---

## 四、建立 EKF3：非线性系统的局部线性化

对于非线性系统 $F(x,u)$，在当前估计状态 $\hat x$ 附近进行一阶 Taylor 展开：

$$
F(x,u)\approx F(\hat x,u)+A(\hat x,u)(x-\hat x)
\tag{21}
$$

其中 $A$ 为状态转移函数的 Jacobian（雅可比）矩阵：

$$
A(x,u)=\frac{\partial F(x,u)}{\partial x}
\tag{22}
$$

展开偏导：

$$
A(x,u)=
\begin{bmatrix}
\dfrac{\partial\dot x_1}{\partial x_1}&\dfrac{\partial\dot x_1}{\partial x_2}&\dfrac{\partial\dot x_1}{\partial x_3}\\[5pt]
\dfrac{\partial\dot x_2}{\partial x_1}&\dfrac{\partial\dot x_2}{\partial x_2}&\dfrac{\partial\dot x_2}{\partial x_3}\\[5pt]
\dfrac{\partial\dot x_3}{\partial x_1}&\dfrac{\partial\dot x_3}{\partial x_2}&\dfrac{\partial\dot x_3}{\partial x_3}
\end{bmatrix}
\tag{23}
$$

由式 (14) 分别求得：

$$
\begin{aligned}
\frac{\partial F_1}{\partial x}&=\begin{bmatrix}0&1&0\end{bmatrix},\\[3pt]
\frac{\partial F_2}{\partial x}&=\begin{bmatrix}
-\dfrac{g}{l}\cos x_1+\dfrac{u}{l}\sin x_1&0&1
\end{bmatrix},\\[3pt]
\frac{\partial F_3}{\partial x}&=\begin{bmatrix}0&0&0\end{bmatrix}.
\end{aligned}
\tag{24}
$$

最终得到 EKF3 的**连续时间状态 Jacobian 矩阵**：

$$
\boxed{
A(\hat x,u)=
\begin{bmatrix}
0&1&0\\[4pt]
-\dfrac{g}{l}\cos\hat\theta+\dfrac{u}{l}\sin\hat\theta&0&1\\[5pt]
0&0&0
\end{bmatrix}}
\tag{25}
$$

视觉测量方程 $h_v(x)=x_1$ 的 Jacobian 为：

$$
\boxed{
H=\frac{\partial h_v}{\partial x}=
\begin{bmatrix}1&0&0\end{bmatrix}}
\tag{26}
$$

---

## 五、EKF3 状态预测与误差协方差预测

在两次视觉测量之间，EKF3 利用已知输入 $u$ 和非线性动力学**预测三个状态**。忽略零均值过程噪声的瞬时取值，有：

$$
\boxed{
\dot{\hat x}=F(\hat x,u)
=\begin{bmatrix}
\hat\omega\\[3pt]
-\dfrac{g}{l}\sin\hat\theta-\dfrac{u}{l}\cos\hat\theta+\hat f\\[5pt]
0
\end{bmatrix}}
\tag{27}
$$

为了描述预测结果的**不确定程度**，定义估计误差及其协方差矩阵：

$$
\boxed{
 e=x-\hat x,\qquad P=\mathbb E[ee^T]
}
\tag{28}
$$

结合第四部分的 Jacobian 矩阵，预测误差在当前估计状态附近满足一阶近似：

$$
\dot e\approx A(\hat x,u)e+Gw
\tag{29}
$$

于是连续时间协方差预测方程为：

$$
\boxed{
\dot P=A(\hat x,u)P+PA(\hat x,u)^T+GQ_cG^T
}
\tag{30}
$$

其中过程噪声贡献为：

$$
GQ_cG^T=
\begin{bmatrix}
0&0&0\\
0&q_\omega&0\\
0&0&q_f
\end{bmatrix}
\tag{31}
$$

因此，在下一帧视觉到来前，**同时传播状态 $\hat x$ 和协方差 $P$**，得到视觉采样时刻的先验估计：

$$
\boxed{
(\hat x_k^-,P_k^-)
=\operatorname{Predict}_{t_{k-1}\to t_k}
(\hat x_{k-1}^+,P_{k-1}^+;u)
}
\tag{32}
$$

这里 $-$ 表示测量校正前，$+$ 表示测量校正后；$\operatorname{Predict}$ 表示求解上述连续时间预测方程，不限定具体数值方法。

---

## 六、视觉测量校正与 Kalman 增益

第 $k$ 帧视觉摆角 $y_k$ 到来时，首先比较视觉测量值和模型预测摆角，定义**残差**：

$$
\boxed{r_k=y_k-H\hat x_k^-=y_k-\hat\theta_k^-}
\tag{33}
$$

残差的不确定性由**预测协方差 $P_k^-$**和**视觉测量噪声方差 $R_k$**共同决定：

$$
\boxed{S_k=HP_k^-H^T+R_k=P_{11,k}^-+R_k}
\tag{34}
$$

通过最小化校正后估计误差的协方差，可得到 **Kalman 增益**：

$$
\boxed{
K_k=P_k^-H^T(HP_k^-H^T+R_k)^{-1}
}
\tag{35}
$$

由于视觉只测量摆角，$H=[1,0,0]$，因此：

$$
\boxed{
K_k=
\frac{1}{P_{11,k}^-+R_k}
\begin{bmatrix}
P_{11,k}^-\\P_{21,k}^-\\P_{31,k}^-
\end{bmatrix}
=\begin{bmatrix}K_{\theta,k}\\K_{\omega,k}\\K_{f,k}\end{bmatrix}
}
\tag{36}
$$

**核心状态校正公式：**

$$
\boxed{\hat x_k^+=\hat x_k^-+K_kr_k}
\tag{37}
$$

展开后三个状态同时得到校正：

$$
\boxed{
\begin{cases}
\hat\theta_k^+=\hat\theta_k^-+K_{\theta,k}r_k,\\[3pt]
\hat\omega_k^+=\hat\omega_k^-+K_{\omega,k}r_k,\\[3pt]
\hat f_k^+=\hat f_k^-+K_{f,k}r_k.
\end{cases}}
\tag{38}
$$

所以，虽然视觉只测量摆角，但通过协方差中的相关项 $P_{21,k}^-$ 和 $P_{31,k}^-$，也能修正摆角速度与扰动估计。

校正状态后，还须更新协方差。采用 Joseph 形式：

$$
\boxed{
P_k^+=(I-K_kH)P_k^-(I-K_kH)^T+K_kR_kK_k^T
}
\tag{39}
$$

至此，EKF3 的**完整工作过程**为：

$$
\boxed{
(\hat x_{k-1}^+,P_{k-1}^+)
\xrightarrow{\text{动力学预测}}
(\hat x_k^-,P_k^-)
\xrightarrow{\text{视觉校正}}
(\hat x_k^+,P_k^+)
}
\tag{40}
$$

> EKF3 以上一时刻的后验状态估计 \(\hat x_{k-1}^{+}\) 和协方差矩阵 \(P_{k-1}^{+}\) 为初始条件，通过非线性动力学模型进行预测，得到当前时刻的先验估计 \((\hat x_k^{-},P_k^{-})\)；随后融合视觉测量值 \(y_k\)，利用 Kalman 增益 \(K_k\) 完成状态与协方差校正，得到后验估计 \((\hat x_k^{+},P_k^{+})\)，并进入下一循环。

![ekf 原理图](<../../images/theory/ekf3.svg>)

---

## 七、EKF3 扩张状态的局部可观测性

在平衡点 $\theta=0,\ \omega=0$ 附近，取名义水平输入 $u=0$，则式 (25) 变为：

$$
\boxed{
A_0=
\begin{bmatrix}
0&1&0\\[3pt]
-\dfrac{g}{l}&0&1\\[4pt]
0&0&0
\end{bmatrix}}
\tag{42}
$$

视觉输出矩阵：

$$
H=\begin{bmatrix}1&0&0\end{bmatrix}
\tag{43}
$$

构造连续时间线性系统的可观测性矩阵：

$$
\mathcal O=
\begin{bmatrix}
H\\HA_0\\HA_0^2
\end{bmatrix}
\tag{44}
$$

分别计算：

$$
HA_0=\begin{bmatrix}0&1&0\end{bmatrix},\qquad
HA_0^2=\begin{bmatrix}-\dfrac{g}{l}&0&1\end{bmatrix}
\tag{45}
$$

所以：

$$
\boxed{
\mathcal O=
\begin{bmatrix}
1&0&0\\
0&1&0\\
-\dfrac{g}{l}&0&1
\end{bmatrix}}
\tag{46}
$$

计算行列式及矩阵秩：

$$
\boxed{\det\mathcal O=1\ne0,\qquad\operatorname{rank}(\mathcal O)=3}
\tag{47}
$$

因此，在上述名义平衡点附近的**连续时间线性化模型**中，三个扩张状态局部可观测。此结论说明状态估计在该局部模型下具有可观测性基础，不等于证明任意采样条件下 EKF3 必然收敛。

---

## 八、状态反馈与扰动补偿：ADRC 控制律推导

EKF3 给出的状态估计：

$$
\boxed{
\hat x=\begin{bmatrix}\hat\theta\\\hat\omega\\\hat f\end{bmatrix}}
\tag{48}
$$

防摆控制目标为 $\theta_{\mathrm{ref}}=0$、$\omega_{\mathrm{ref}}=0$。选择期望摆角闭环动力学：

$$
\boxed{\ddot\theta_{\mathrm{des}}=-k_p\theta-k_d\omega}
\tag{49}
$$

令虚拟角加速度控制量为：

$$
\boxed{v=-k_p\hat\theta-k_d\hat\omega}
\tag{50}
$$

由非线性吊挂模型式 (7)：

$$
\ddot\theta=-\frac{g}{l}\sin\theta
-\frac{a}{l}\cos\theta+f
\tag{51}
$$

希望实际摆角加速度实现虚拟控制目标，使用 EKF3 估计量代替未知状态和扰动：

$$
-\frac{g}{l}\sin\hat\theta
-\frac{a_{\mathrm{anti}}}{l}\cos\hat\theta+\hat f
=-k_p\hat\theta-k_d\hat\omega
\tag{52}
$$

将基座加速度项移至左侧：

$$
\frac{a_{\mathrm{anti}}}{l}\cos\hat\theta
=k_p\hat\theta+k_d\hat\omega
-\frac{g}{l}\sin\hat\theta+\hat f
\tag{53}
$$

当 $\cos\hat\theta\ne0$ 时，得到**非线性 ADRC 抗摆加速度控制律**：

$$
\boxed{
a_{\mathrm{anti}}=
\frac{
l k_p\hat\theta
+l k_d\hat\omega
-g\sin\hat\theta
+l\hat f
}{\cos\hat\theta}
}
\tag{54}
$$

式 (54) 中分别包含摆角反馈、摆角速度反馈、重力非线性项补偿及等效扰动补偿。

### ADRC 控制器带宽法增益配置

对于理想期望闭环：

$$
\ddot\theta+k_d\dot\theta+k_p\theta=0
\tag{55}
$$

特征方程为：

$$
s^2+k_d s+k_p=0
\tag{56}
$$

取期望二阶系统的自然频率 $\omega_c>0$、阻尼比 $\zeta>0$：

$$
s^2+2\zeta\omega_c s+\omega_c^2=0
\tag{57}
$$

逐项比较得到：

$$
\boxed{k_p=\omega_c^2,\qquad k_d=2\zeta\omega_c}
\tag{58}
$$

当 $\zeta=1$ 时，期望闭环具有重复实极点 $s=-\omega_c$；当 $0<\zeta<1$ 时为欠阻尼系统。

### 名义闭环验证

在状态与扰动估计准确，实际悬挂点加速度能够跟踪 $a_{\mathrm{anti}}$，且 $\cos\theta\ne0$ 的理想条件下：

$$
\hat\theta=\theta,\qquad
\hat\omega=\omega,\qquad
\hat f=f,\qquad
a=a_{\mathrm{anti}}
\tag{59}
$$

将式 (54) 代入真实动力学式 (51)，得到：

$$
\boxed{\ddot\theta+k_d\dot\theta+k_p\theta=0}
\tag{60}
$$

定义正定函数：

$$
V(\theta,\omega)=\frac12\omega^2+\frac12k_p\theta^2
\tag{61}
$$

沿名义闭环轨迹求导：

$$
\begin{aligned}
\dot V
&=\omega\dot\omega+k_p\theta\dot\theta\\
&=\omega(-k_d\omega-k_p\theta)+k_p\theta\omega\\
&=\boxed{-k_d\omega^2\le0}.
\end{aligned}
\tag{62}
$$

当 $k_p>0$、$k_d>0$ 时，式 (60) 的线性闭环特征根均位于复平面左半平面，因此**理想等效闭环**指数稳定。上述结果不等同于对含估计误差、执行器限制或额外飞行控制环的完整实际系统作出稳定性保证。

---

## 九、东西 / 南北双平面表达

对 East 和 North 两个水平轴，分别定义独立的三维扩张状态：

$$
\boxed{
x_E=\begin{bmatrix}\theta_E\\\omega_E\\f_E\end{bmatrix},
\qquad
x_N=\begin{bmatrix}\theta_N\\\omega_N\\f_N\end{bmatrix}
}
\tag{63}
$$

对应的非线性模型：

$$
\boxed{
\begin{cases}
\ddot\theta_E=-\dfrac{g}{l}\sin\theta_E-\dfrac{a_E}{l}\cos\theta_E+f_E,\\[6pt]
\ddot\theta_N=-\dfrac{g}{l}\sin\theta_N-\dfrac{a_N}{l}\cos\theta_N+f_N.
\end{cases}}
\tag{64}
$$

各轴应用式 (13)–(47) 的 EKF3 预测与视觉校正，得到：

$$
\hat x_E=\begin{bmatrix}\hat\theta_E\\\hat\omega_E\\\hat f_E\end{bmatrix},
\qquad
\hat x_N=\begin{bmatrix}\hat\theta_N\\\hat\omega_N\\\hat f_N\end{bmatrix}
\tag{65}
$$

再将每个轴的估计状态代入式 (54)：

$$
\boxed{
a_{\mathrm{anti},i}=
\frac{
l k_p\hat\theta_i+l k_d\hat\omega_i-g\sin\hat\theta_i+l\hat f_i
}{\cos\hat\theta_i},\quad i\in\{E,N\}
}
\tag{66}
$$

至此完成**吊挂非线性动力学 → 三维扩张状态 EKF → 视觉测量校正 → ADRC 抗摆加速度控制**的理论推导。

<!-- Codex-generated: current football scene configuration and verified test procedure. -->
# 足球场与起飞台

默认场景为足球场和原点起飞台，统一使用 `sim/worlds/UAV_slung_world.sdf` 与同目录 `gui.config`。原篮球场模型和重复的旧场景已从工程移除。

## 布局

| 项目 | 当前配置（世界坐标） |
|---|---|
| 起飞台中心 | `(0, 0, 0)` |
| 台面高度 | `1.072 m` |
| 起飞台旋转 | 绕 Z 轴 `-90°`，H 字开口朝向大门 |
| 初始机头方向 | `+X`，即 ENU 的东向 |
| 门框中心 | `(15, 0)`，从台心向前 15 m |
| 大门 | 净宽 9 m、净高 4 m；门扇 3 列×2 行，向外开启 160°，与围栏夹角 20° |
| 球场中心 | `(54, 0)` |
| 白线内范围 | `X=20..88 m`、`Y=±52.5 m`，105×68 m |
| 围栏范围 | `X=15..93 m`、`Y=±57.5 m` |
| 原点辅助网格 | 默认 20×20 m、每格 1 m，仅在 GUI 显示 |

起飞台已完整复制至 `sim/models/helipad`，包括模型、网格、材质、H 和角标及碰撞体；运行不依赖 `UAV_lumberjack` 的模型路径。原资产来源记录在该目录 `SOURCE.md`。无人机、吊绳和载荷的出生高度整体提高了 1.072 m，保持原相对连接关系。

原点网格和 H 字只用于定位，不会自动生成飞行任务。

## 外观与光照

- 草地由三种草纹连续融合为两张 2K 深浅贴图，按 7 m 条带交替；只保留当前使用的贴图。
- 四面角旗为红黄交错四格，静态轻微波形。
- 灯杆和长椅网格已独立放在足球场模型目录，照明灯位于笼内四角的白线外空地。
- 环境光为中性柔光，方向光强度 0.70、路灯强度 0.18；外部地面使用哑光材质，抑制高光。
- GUI 沿用原浅灰色工具栏配色；初始视角展示起飞台和大门。
- 外部地面与球场地坪的可视区域分开，避免共面闪烁。

球网仅作可视参考，不参与碰撞。围栏、门和场边设施使用简化碰撞体，门扇固定开启，不带交互开关控制。

## 启动

在每个终端加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source ~/UAV_slung/ros2_ws/install/setup.bash
```

打开 QGroundControl，分别在三个终端运行：

```bash
# 终端 1：Gazebo、PX4、DDS 和视觉检测
ros2 launch slung_ctrl sim.launch.py

# 终端 2：视觉、真值和观测器
ros2 launch slung_ctrl estimation.launch.py estimator:=all

# 终端 3：交互飞行控制
ros2 launch slung_ctrl offboard.launch.py
```

无需为场景资源重新编译 ROS 包。安装的启动文件通过符号链接加载源码。

## 穿门往返测试

控制终端的 `setpoint` 使用相对起飞点的 **NORTH EAST UP** 坐标，初始朝向的大门位于 East 正方向。默认 `takeoff` 高度为相对起飞点 8 m；穿门前需要下降到测试高度。

依次输入，每一步等待位置稳定：

```text
takeoff
setpoint 0 0 2
setpoint 0 18 2
setpoint 0 0 2
home
land
```

这里 `UP=2` 是相对起飞点的高度，机体模型原点约处于世界 Z=3.1 m，低于门楣；`EAST=18` 会经过 X=15 m 的门，进入笼内。实际飞行仍应观察载荷、位置和视觉反馈。现有控制器及观测器参数未因本次场景整理而修改。

## 检查与冻结候选

```bash
SDF_PATH="$HOME/UAV_slung/sim/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models" \
  gz sdf -k ~/UAV_slung/sim/worlds/UAV_slung_world.sdf
```

本轮验收结果和候选快照见 `docs/scene_validation_20261026.md`。该文件记录实际执行的检查，不把场景快照当作整个仓库的 Git 冻结。

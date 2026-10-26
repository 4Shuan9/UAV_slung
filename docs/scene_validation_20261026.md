<!-- Codex-generated: final scene validation and cleanup record. -->
# 场景冻结候选验收：2026-10-26

状态：本轮场景调整、资源清理和一次完整仿真往返验收通过，可供用户冻结。

## 当前场景

- 起飞台已完整复制到 `sim/models/helipad`，不依赖 UAV_lumberjack 的运行资源路径。
- 起飞台保留在原点，旋转 -90°，H 字开口对着 +X 方向的大门；门中心为 (15,0)。
- 原点 GUI 网格恢复为默认的 20×20 m、1 m 间距。
- 使用中性柔光、低强度方向光与路灯、哑光地面；原灰色 GUI 配色保留。
- 场景统一到 `sim/worlds/UAV_slung_world.sdf` 和 `sim/worlds/gui.config`。

## 本次实际执行的验收

1. SDFormat 完整世界检查通过；场景资源路径审计通过。
2. 起飞台、15 m 门框距离、H 开口朝向、光照和地面的实际渲染检查通过。
3. GUI 应用实际加载检查通过，工具栏仍为 #f3f3f3 / #111111。
4. 使用 ROS 域 79、独立 Gazebo 通信分区、DDS 端口 18888 和临时 PX4 运行目录进行纯仿真测试。通过本地 GCS 心跳正常完成预飞检查，未关闭预飞检查或强制解锁。
5. 使用现有 ESO-delay + ADRC 配置完成起飞至 8 m、下降到相对起飞点 2 m、穿过门到 East=18 m、返回、回到 8 m、降落和正常解除解锁。控制器及观测器参数未因本轮场景调整修改。

### 仿真检查点（Gazebo 世界坐标）

| 检查点 | X / m | Y / m | Z / m | armed |
|---|---:|---:|---:|---|
| ready | 0.038 | -0.000 | 1.071 | False |
| takeoff_hover | 0.044 | 0.017 | 9.099 | True |
| approach_height | 0.031 | 0.030 | 3.307 | True |
| inside_gate | 17.688 | -0.008 | 3.077 | True |
| returned_low | 0.358 | -0.052 | 3.087 | True |
| home_hover | 0.059 | -0.031 | 8.777 | True |
| landed_disarmed | 0.062 | 0.036 | 1.074 | False |

从就绪到降落解除解锁约 138.8 秒，期间收到了 3724 帧相机图像。最终机体模型原点 Z=1.0745 m；平台承载面为 1.072 m。落点距台心约 7.2 cm。验收检查点的 PX4 failsafe 标志均为 false。

该结论覆盖本次环境与一次往返测试，不代表所有天气、速度、载荷和轨迹工况均已验证。

## 清理范围

- 移除本工程 `sim/models/basketball_court`。
- 用当前足球场替换根目录旧篮球场世界；移除重复的 `sim/worlds/football` 子目录。
- 移除本轮生成、已停用的两张 512px 草地贴图；保留当前使用的两张 2K 融合贴图。
- 更新 README 的场景功能说明及当前场景使用指南。
- 用户维护的其他模型、控制源码和参数不属于本次清理目标。

## 候选快照

本轮验收结果、资源审计、图片和候选源码压缩包保存在：

`/home/ashuang/文档/ChatGPT/UAV_slung/reports/scene_freeze_candidate_20261026`

候选源码包包含当前工作区的 sim、ROS 源码、tools、docs、images 与主要项目说明，不包含 build/install/log 和 Git 元数据。文件校验值记录在外部 `manifest.json`，并随压缩包保存。已有工作区修改会按当前内容进入快照；这不是自动创建的 Git 提交或标签。

启动命令：

```bash
ros2 launch slung_ctrl sim.launch.py
```

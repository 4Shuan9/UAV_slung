<!-- Codex-generated: consolidated model, texture and project source references, 2026-10-27. -->
# 模型、贴图与项目来源

本文件集中记录场景素材来源及相关项目。仿真使用的文件保留在 `sim/models` 下各自的模型目录。 本轮已将足球场目录统一为 `football_field`，同步 SDF 与网格材质引用；资源来源不因重命名改变。

## 场景模型

| 素材 | 来源与用途 | 工程内文件 |
|---|---|---|
| 足球场 | 用户提供的 `football_105x68_basket_exactstyle_gate_v2.sdf`，经过合并和场景调整。原始 SDF 目前未找到 | `football_field/model.sdf` |
| 球门框、场灯 | [Low Poly Running Race Track](https://sketchfab.com/3d-models/low-poly-running-race-track-game-ready-asset-23ec0c80500e4269976b6e434fab2f67)，Jackson.Charles，CC BY；从用户提供的 GLB 提取并转换 | `football_field/meshes/stadium_goal.dae`、`stadium_light.dae` |
| 三排看台 | [Bleachers three row](https://www.innerscene.com/tools/library/3d-parts/bleachers-three-row-sports-equipment-7221c461)，Innerscene，CC0；从用户提供的 GLB 转换 | `football_field/meshes/spectator_bleachers.dae`；贴图集中在该模型 `materials/textures/`，按钢、铝、木材命名 |
| 起飞台 | [UAV_lumberjack](https://github.com/4Shuan9/UAV_lumberjack)，从 `~/UAV_lumberjack/sim/models/helipad` 复制；位于世界原点，顶面碰撞高度 1.072 m | `helipad/` |
| 球员座椅 | 从本工程此前的篮球场模型中提取 | `football_field/meshes/player_bench.dae` |
| 单门、双门 | 工程既有资源，`model.config` 登记作者为 skn256；原始下载页与许可尚未核实 | `single_gate/`、`double_gate/` |
| 单目相机 | 工程既有模型，`model.config` 登记作者为 Markus Achtelik；采用本工程的相机配置 | `mono_cam/` |
| 绳索、吊球 | 工程既有 SDF 模型；几何、质量和惯量以当前基准配置为准 | `rope/`、`payload_ball/` |

球门网、护网、角旗、圆形地面和边框为当前场景生成或改制的网格。外部模型经提取、缩放或转换，实际尺寸和材质以工程内运行文件为准。

## 场景贴图

- 草坪：[Grass001](https://ambientcg.com/view?id=Grass001)、[Grass004](https://ambientcg.com/view?id=Grass004)、[Grass008](https://ambientcg.com/view?id=Grass008)，ambientCG，CC0。三张 1K 颜色图经调色、旋转和平滑周期混合，生成 2K 明暗两种贴图；重复尺度约 2.8 m，草坪条纹宽约 7 m。运行文件位于 `football_field/materials/textures/`。
- 外部地面：[Gravel Embedded Concrete](https://polyhaven.com/a/gravel_embedded_concrete)，Charlotte Baglioni / Poly Haven，CC0。以原 2K 颜色图为基础，拼接生成 4K 贴图并模拟 4 m×4 m 接缝；仅改变颜色和边缘明暗，不增加沟槽或碰撞。运行文件为 `stadium_ground/materials/textures/concrete_tiles_4k.jpg`。

- 看台：随上述三排看台 GLB 一并提取；钢、铝和木材贴图分别为 `football_field/materials/textures/bleacher_steel.png`、`bleacher_aluminum.png`、`bleacher_wood.png`。本轮仅按材质重命名并迁移目录，像素内容未变。
- 备用门框：`single_gate/materials/textures/a2rl_gate.png`、`double_gate/materials/textures/a2rl_gate.png`，随原门框模型保留；本轮修正 DAE 的相对引用路径，未更改贴图内容。

当前草坪和水泥地使用颜色贴图，没有加载法线或位移贴图。未核实许可的既有资源不在此推定为 CC0 或其他授权。

## 仿真与飞控基础项目

按飞控、通信、仿真和地面站排序；本工程环境版本以根目录 `README.md` 为准。

1. [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) — PX4 飞控和 SITL。
2. [px4_msgs](https://github.com/PX4/px4_msgs) — PX4 对应的 ROS 2 消息定义。
3. [px4_ros_com](https://github.com/PX4/px4_ros_com) — PX4 与 ROS 通信相关项目及示例。
4. [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) — PX4 与 DDS 网络之间的通信代理。
5. [gz-sim](https://github.com/gazebosim/gz-sim) — Gazebo Sim 仿真器。
6. [QGroundControl](https://github.com/mavlink/qgroundcontrol) — 地面站。

## 相关项目与模型库

1. [UAV_lumberjack](https://github.com/4Shuan9/UAV_lumberjack) — 本人的相关工程，起飞台来源。
2. [PX4-ROS2-Gazebo-Drone-Simulation-Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template) — PX4、ROS 2 和 Gazebo 无人机仿真模板。
3. [rain_ws](https://github.com/RainbowSeeker/rain_ws) — 用户提供的参考仓库；当前未能核实仓库内容。
4. [gazebo_models](https://github.com/osrf/gazebo_models) — Gazebo 模型库，Gazebo Fuel 模型库的前身。

除已明确说明的起飞台等素材来源外，列出参考项目不表示本工程直接依赖或复制了其代码。

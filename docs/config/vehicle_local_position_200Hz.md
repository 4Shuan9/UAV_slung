# PX4：vehicle_local_position 话题频率配置 100 Hz → 200 Hz

目标话题：`/fmu/out/vehicle_local_position`，加速度字段为 `ax、ay、az`。

---

## 1. 修改 DDS 转发间隔

打开文件：`~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.h.em`

将：

```cpp
#define UXRCE_DEFAULT_POLL_RATE 10
```

改为：

```cpp
#define UXRCE_DEFAULT_POLL_RATE 5
```

（单位：ms）

重新编译 PX4：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default
```

---

## 2. 在 QGroundControl 修改参数

- 进入“车辆设置 → 参数”
- 搜索 `EKF2_PREDICT_US`
- 将 `10000` 改为 `5000`（单位：μs）

---

## 3. 验证并检查重启保留

```bash
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-param --instance 0 show EKF2_PREDICT_US
ros2 topic hz /fmu/out/vehicle_local_position
```

确认参数为 `5000`，话题平均频率接近 `200 Hz`。

重启仿真后检查。

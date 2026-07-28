# BAAI Go2 自主导航

Unitree Go2 + Livox MID360 的 ROS Noetic 三维定位与二维自主导航稳定版配置。


<img width="4032" height="3024" alt="20260728-135047" src="https://github.com/user-attachments/assets/205d0204-85d7-4dc1-8fac-89b23b2354ac" />
<img width="5712" height="4284" alt="20260728-135246" src="https://github.com/user-attachments/assets/f694b787-e93b-42ab-a095-e4030d31c71f" />



## 当前功能

- FAST-LIO 实时点云与 Open3D PLY 全局定位
- 保留 PLY 原始 RGB 颜色的三维地图显示
- Z-up 三维地图投影为固定 `/map_grid`
- RViz 3D 地图、实时雷达、全局路径和局部路径显示
- 修正 `go2_link` 固定 90° 航向偏差
- TEB 与 Unitree 运动桥统一限速
- 启动时自动取消旧目标并保持运动桥关闭

## 机器人端文件位置

将仓库中的文件部署到以下位置：

| 仓库文件 | 机器人路径 |
| --- | --- |
| `scripts/start_xlc_navigation.sh` | `/home/unitree/start_xlc_navigation.sh` |
| `scripts/xlc_planar_base_tf.py` | `/home/unitree/xlc_planar_base_tf.py` |
| `rviz/xlc_navigation.rviz` | `/home/unitree/xlc_navigation.rviz` |
| `localization/launch/open3d_loc_g1.launch` | `FAST_LIO_LOCALIZATION_HUMANOID/.../open3d_loc/launch/open3d_loc_g1.launch` |
| `localization/src/global_localization.cpp` | `FAST_LIO_LOCALIZATION_HUMANOID/.../open3d_loc/src/global_localization.cpp` |
| `navigation/octomap_projector/projector.launch` | `go2_navigation250917251016/src/octomap_projector/launch/projector.launch` |
| `navigation/robot_navigation/launch/navigationcopy2.launch` | `go2_navigation250917251016/src/robot_navigation/launch/navigationcopy2.launch` |
| `navigation/robot_navigation/config/*` | `go2_navigation250917251016/src/robot_navigation/config/` |
| `unitree_bridge/go2_move_sub.cpp` | `go_ros_sdk/gogo/unitree_sdk2/example/go2/go2_move_sub.cpp` |

地图文件不进入 Git。当前启动配置期望机器人上存在：

```text
/home/unitree/FAST_LIO_LOCALIZATION_HUMANOID/src/FAST_LIO_LOCALIZATION_HUMANOID/data/baai0723_21_32_58_zup.ply
```

## 使用

启动雷达、定位、导航和 RViz（运动默认关闭）：

```bash
/home/unitree/start_xlc_navigation.sh start
```

检查状态：

```bash
/home/unitree/start_xlc_navigation.sh status
```

确认机器人稳定站立、处于行走模式且 Unitree App 已退出后启用运动：

```bash
/home/unitree/start_xlc_navigation.sh enable-motion
```

紧急停止运动：

```bash
/home/unitree/start_xlc_navigation.sh disable-motion
```

停止全部模块：

```bash
/home/unitree/start_xlc_navigation.sh stop
```

## 运动参数

- 最大前进速度：`0.50 m/s`
- 最大后退速度：`0.15 m/s`
- 最大角速度：`0.30 rad/s`
- 启用运动前最低定位置信度：`0.30`

首次测试请使用前方 0.5–1 米的空旷近距离目标。

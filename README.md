## 系统概述

这套点云地图构建系统由以下几个主要组件组成：

1. **动态点过滤器** (`dynamic_point_remover.py`) - 移除动态物体点云
2. **地图构建器** (`pointcloud_map_builder.py`) - 累积静态点云生成地图
3. **地图工具** (`map_utils.py`) - 地图分析和处理工具

## 快速开始

### 1. 启动动态点过滤器
```bash
# 基本启动
roslaunch your_package dynamic_point_remover.launch

# 或使用自定义参数
rosrun your_package dynamic_point_remover.py _pointcloud_topic:=/your_pointcloud _marker_topic:=/your_markers
```

### 2. 启动地图构建器
```bash
# 时间间隔模式（每2秒保存一帧）
roslaunch your_package pointcloud_map_builder.launch

# 帧数间隔模式（每5帧保存一帧）
rosrun your_package pointcloud_map_builder.py _save_mode:=frame _frame_interval:=5
```

### 3. 可视化
```bash
# 启动RViz
rosrun rviz rviz -d /path/to/map_builder.rviz

# 或在启动时直接加载
roslaunch your_package pointcloud_map_builder.launch rviz:=true
```

## 主要功能

### 地图构建模式

1. **时间间隔模式** (`save_mode: "time"`)
   - 按固定时间间隔保存点云帧
   - 适合匀速移动的场景
   - 参数: `time_interval` (秒)

2. **帧数间隔模式** (`save_mode: "frame"`)
   - 按固定帧数间隔保存点云帧
   - 适合变速移动的场景
   - 参数: `frame_interval` (帧数)

### 地图质量控制

- **体素滤波**: 减少点云密度，提高处理效率
- **距离滤波**: 移除过远的噪声点
- **点数阈值**: 忽略点数过少的帧
- **地图大小限制**: 防止内存溢出

### 服务接口

```bash
# 手动保存地图
rosservice call /pointcloud_map_builder/save_map

# 清空当前地图
rosservice call /pointcloud_map_builder/clear_map
```

## 地图工具使用

### 分析地图
```bash
python3 map_utils.py analyze --input /path/to/map.pkl
```

### 发布地图供可视化
```bash
python3 map_utils.py publish --input /path/to/map.pkl --frame map
```

### 裁剪地图
```bash
python3 map_utils.py crop --input map.pkl --output cropped_map.pkl \
    --x-min -10 --x-max 10 --y-min -5 --y-max 5 --z-min 0 --z-max 3
```

### 合并多个地图
```bash
python3 map_utils.py merge --files map1.pkl map2.pkl map3.pkl --output merged_map.pkl
```

### 转换为PCD格式
```bash
python3 map_utils.py convert --input map.pkl --output map.pcd
```

## 参数调优建议

### 性能优化
- 增大 `voxel_size` 以减少点云密度
- 增大 `time_interval` 或 `frame_interval` 以减少处理频率
- 设置合适的 `max_map_points` 限制内存使用

### 精度优化
- 减小 `voxel_size` 以保留更多细节
- 减小 `expansion_margin` 以减少过度过滤
- 调整 `min_points_threshold` 过滤噪声帧

### 存储优化
- 启用 `auto_save` 定期保存地图
- 设置合适的 `save_interval` 平衡性能和安全性
- 选择合适的 `save_directory` 确保足够存储空间

## 故障排除

### 常见问题

1. **TF变换失败**
   - 检查坐标系设置是否正确
   - 确保TF树完整
   - 调整 `transform_timeout` 参数

2. **地图点数过多**
   - 增大 `voxel_size`
   - 减小 `max_distance`
   - 增大采样间隔

3. **地图质量差**
   - 检查动态物体过滤效果
   - 调整体素滤波参数
   - 优化传感器标定

### 监控命令
```bash
# 查看话题信息
rostopic info /filtered_pointcloud
rostopic info /accumulated_map

# 监控处理频率
rostopic hz /filtered_pointcloud

# 查看节点状态
rosnode info /pointcloud_map_builder
```

# 文件结构说明
# 推荐的包文件结构:

pointcloud_mapping/
├── CMakeLists.txt
├── package.xml
├── README.md
├── scripts/
│   ├── dynamic_point_remover.py
│   ├── pointcloud_map_builder.py
│   └── map_utils.py
├── launch/
│   ├── dynamic_point_remover.launch
│   └── pointcloud_map_builder.launch
├── config/
│   └── map_builder_config.yaml
├── rviz/
│   └── map_builder.rviz
└── docs/
    └── usage_guide.md

# 安装和编译步骤:

1. 将代码放到catkin工作空间的src目录下:
   cd ~/catkin_ws/src
   git clone <your_repo> pointcloud_mapping

2. 确保Python脚本有执行权限:
   chmod +x pointcloud_mapping/scripts/*.py

3. 编译包:
   cd ~/catkin_ws
   catkin_make

4. 设置环境变量:
   source devel/setup.bash

5. 运行节点:
   roslaunch pointcloud_mapping pointcloud_map_builder.launch

# 依赖安装:
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    python3-numpy
## 系统概述

这套点云地图构建系统由以下几个主要组件组成：

1. **动态物体检测** https://github.com/ethz-asl/dynablox.git
2. **动态点过滤器** (`dynamic_point_remover.py`) - 移除动态物体点云
3. **地图构建器** (`pointcloud_map_builder.py`) - 累积静态点云生成地图


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

### 地图保存

- **手动保存**: 调用服务保存当前累积的所有点云为PCD文件
- **自动保存**: 程序退出时自动保存
- **文件格式**: 标准PCD格式，便于使用PCL等工具处理

### 地图质量控制

- **体素滤波**: 减少点云密度，提高处理效率
- **距离滤波**: 移除过远的噪声点
- **点数阈值**: 忽略点数过少的帧
- **地图大小限制**: 防止内存溢出

### 服务接口

```bash
# 手动保存地图（保存为PCD文件）
rosservice call /pointcloud_map_builder/save_map

# 清空当前地图
rosservice call /pointcloud_map_builder/clear_map
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
- 设置合适的 `save_directory` 确保足够存储空间
- 选择合适的 `map_name` 便于文件管理
- PCD格式便于后续处理和可视化

## 使用流程

### 基本操作

1. **启动系统**:
```bash
roslaunch pointcloud_mapping pointcloud_map_builder.launch
```

2. **可视化监控**:
```bash
rosrun rviz rviz -d pointcloud_mapping/rviz/map_builder.rviz
```

3. **保存地图**:
```bash
# 手动保存（任何时候）
rosservice call /pointcloud_map_builder/save_map

# 程序会在退出时自动保存
```

4. **使用保存的地图**:
```bash
# 使用PCL工具查看
pcl_viewer /path/to/your_map_20240815_143022.pcd

# 或在RViz中加载PCD文件
```

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
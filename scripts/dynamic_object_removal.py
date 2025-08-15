#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
3D激光雷达动态物体去除和静态地图构建工具
基于多帧统计、体素占用分析和范围图像对比的综合方法
实时订阅模式版本
"""

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict, deque
import open3d as o3d
import time
import argparse
import os
import sys
import yaml
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
import warnings
warnings.filterwarnings('ignore')

class VoxelGrid:
    """体素网格类，用于高效的空间索引和占用统计"""
    def __init__(self, voxel_size=0.1):
        self.voxel_size = voxel_size
        self.voxels = defaultdict(lambda: {
            'count': 0,
            'dynamic_count': 0,
            'points': [],
            'timestamps': [],
            'occupancy_history': deque(maxlen=20)
        })
    
    def get_voxel_key(self, point):
        """获取点的体素键值"""
        return tuple(np.floor(point[:3] / self.voxel_size).astype(int))
    
    def add_point(self, point, timestamp, is_dynamic=False):
        """添加点到体素"""
        key = self.get_voxel_key(point)
        self.voxels[key]['points'].append(point)
        self.voxels[key]['timestamps'].append(timestamp)
        self.voxels[key]['count'] += 1
        if is_dynamic:
            self.voxels[key]['dynamic_count'] += 1
        self.voxels[key]['occupancy_history'].append(1)
        return key
    
    def get_dynamic_ratio(self, key):
        """获取体素的动态比率"""
        if key not in self.voxels or self.voxels[key]['count'] == 0:
            return 0
        return self.voxels[key]['dynamic_count'] / self.voxels[key]['count']
    
    def get_occupancy_variance(self, key):
        """获取体素占用方差（用于检测变化）"""
        if key not in self.voxels:
            return 0
        history = list(self.voxels[key]['occupancy_history'])
        if len(history) < 2:
            return 0
        return np.var(history)

class RangeImage:
    """范围图像类，用于基于投影的动态检测"""
    def __init__(self, width=1800, height=64, fov_up=15.0, fov_down=-15.0):
        self.width = width
        self.height = height
        self.fov_up = np.deg2rad(fov_up)
        self.fov_down = np.deg2rad(fov_down)
        self.fov_vertical = self.fov_up - self.fov_down
        
    def points_to_range_image(self, points):
        """将点云转换为范围图像"""
        range_image = np.full((self.height, self.width), -1, dtype=np.float32)
        
        # 计算每个点的球坐标
        depths = np.linalg.norm(points[:, :3], axis=1)
        scan_x = points[:, 0]
        scan_y = points[:, 1]
        scan_z = points[:, 2]
        
        # 计算角度
        yaw = np.arctan2(scan_y, scan_x)
        pitch = np.arcsin(scan_z / (depths + 1e-10))
        
        # 转换到图像坐标
        proj_x = (yaw + np.pi) / (2 * np.pi) * self.width
        proj_y = (1.0 - (pitch - self.fov_down) / self.fov_vertical) * self.height
        
        proj_x = np.floor(proj_x).astype(np.int32)
        proj_y = np.floor(proj_y).astype(np.int32)
        
        # 边界检查
        valid_mask = (proj_x >= 0) & (proj_x < self.width) & \
                    (proj_y >= 0) & (proj_y < self.height)
        
        proj_x = proj_x[valid_mask]
        proj_y = proj_y[valid_mask]
        depths = depths[valid_mask]
        
        # 填充范围图像
        for px, py, d in zip(proj_x, proj_y, depths):
            if range_image[py, px] < 0 or d < range_image[py, px]:
                range_image[py, px] = d
                
        return range_image
    
    def compare_range_images(self, img1, img2, threshold=1.0):
        """比较两个范围图像，返回差异掩码"""
        valid_mask = (img1 > 0) & (img2 > 0)
        diff = np.abs(img1 - img2)
        dynamic_mask = valid_mask & (diff > threshold)
        return dynamic_mask

class DynamicObjectRemover:
    def __init__(self, config=None):
        """初始化动态物体去除器"""
        # 加载配置
        self.load_config(config)
        
        # 初始化组件
        self.voxel_grid = VoxelGrid(self.voxel_size)
        self.range_image_processor = RangeImage()
        
        # 数据存储
        self.frame_buffer = deque(maxlen=self.observation_window)
        self.pose_buffer = deque(maxlen=self.observation_window)
        self.static_map_points = []
        self.processed_frames = 0
        self.start_time = time.time()
        
        # TF缓冲区
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(3600))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布器
        self.setup_publishers()
        
        # 订阅点云话题
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback, queue_size=1)
        
        # 服务
        self.save_service = rospy.Service('save_static_map', Trigger, self.handle_save_map)
        
        # Fake TF广播（如果需要）
        if self.use_fake_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self.timer = rospy.Timer(rospy.Duration(0.05), self.broadcast_fake_tf)
        
        rospy.loginfo("=== Dynamic Object Remover Initialized ===")
        rospy.loginfo(f"Subscribing to: {self.pointcloud_topic}")
        self.print_config()
    
    def broadcast_fake_tf(self, event):
        """发布一个假的TF变换"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.lidar_frame

        # 设置固定变换
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
    
    def load_config(self, config_file=None):
        """加载配置参数"""
        # 默认参数
        defaults = {
            'voxel_size': 0.1,
            'min_observations': 3,
            'dynamic_threshold': 0.3,
            'ground_height_threshold': 0.3,
            'max_range': 50.0,
            'min_range': 0.5,
            'observation_window': 10,
            'cluster_tolerance': 0.5,
            'min_cluster_size': 10,
            'use_range_image': True,
            'use_statistical_filter': True,
            'statistical_mean_k': 50,
            'statistical_std_dev': 1.0,
            'pointcloud_topic': '/velodyne_points',
            'map_frame': 'map',
            'lidar_frame': 'velodyne',
            'use_fake_tf': False
        }
        
        # 从ROS参数服务器加载
        for key, default_value in defaults.items():
            setattr(self, key, rospy.get_param(f'~{key}', default_value))
        
        # 如果提供了配置文件，从文件加载
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                for key, value in config.items():
                    if hasattr(self, key):
                        setattr(self, key, value)
    
    def print_config(self):
        """打印配置信息"""
        rospy.loginfo("Configuration:")
        rospy.loginfo(f"  - Voxel size: {self.voxel_size} m")
        rospy.loginfo(f"  - Dynamic threshold: {self.dynamic_threshold}")
        rospy.loginfo(f"  - Observation window: {self.observation_window} frames")
        rospy.loginfo(f"  - Range: [{self.min_range}, {self.max_range}] m")
        rospy.loginfo(f"  - Use range image: {self.use_range_image}")
        rospy.loginfo(f"  - Map frame: {self.map_frame}")
        rospy.loginfo(f"  - LiDAR frame: {self.lidar_frame}")
        rospy.loginfo(f"  - Use fake TF: {self.use_fake_tf}")
    
    def setup_publishers(self):
        """设置ROS发布器"""
        self.static_cloud_pub = rospy.Publisher(
            '/static_pointcloud', PointCloud2, queue_size=1)
        self.dynamic_cloud_pub = rospy.Publisher(
            '/dynamic_pointcloud', PointCloud2, queue_size=1)
        self.map_cloud_pub = rospy.Publisher(
            '/map_pointcloud', PointCloud2, queue_size=1)
        self.ground_cloud_pub = rospy.Publisher(
            '/ground_pointcloud', PointCloud2, queue_size=1)
    
    def pointcloud_callback(self, cloud_msg):
        """点云回调函数"""
        self.process_single_frame(cloud_msg)
    
    def process_single_frame(self, cloud_msg):
        """处理单帧点云"""
        try:
            # 1. 转换点云格式
            points = self.pointcloud2_to_numpy(cloud_msg)
            if points is None or len(points) == 0:
                return
            
            # 2. 获取变换
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, cloud_msg.header.frame_id, 
                    cloud_msg.header.stamp, rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5, f"TF lookup failed: {e}")
                return
            
            # 3. 变换到地图坐标系
            points_map = self.transform_pointcloud(points, transform)
            
            # 4. 范围过滤
            points_map = self.filter_by_range(points_map)
            
            # 5. 地面分割
            ground_points, non_ground_points = self.segment_ground(points_map)
            
            # 6. 动态物体检测
            if len(self.frame_buffer) >= 2:
                static_mask, dynamic_mask = self.detect_dynamic_objects(
                    non_ground_points, ground_points)
                
                static_points = non_ground_points[static_mask]
                dynamic_points = non_ground_points[dynamic_mask]
                
                # 更新体素网格
                self.update_voxel_grid(static_points, dynamic_points, 
                                      cloud_msg.header.stamp.to_sec())
                
                # 发布当前帧结果
                self.publish_frame_results(
                    static_points, dynamic_points, ground_points, 
                    cloud_msg.header.stamp)
            else:
                # 初始帧，全部视为静态
                self.update_voxel_grid(non_ground_points, np.array([]), 
                                      cloud_msg.header.stamp.to_sec())
            
            # 7. 更新缓冲区
            self.frame_buffer.append(non_ground_points)
            self.pose_buffer.append(transform)
            
            self.processed_frames += 1
            
            # 进度报告
            if self.processed_frames % 50 == 0:
                elapsed = time.time() - self.start_time
                fps = self.processed_frames / elapsed if elapsed > 0 else 0
                rospy.loginfo(f"Processed {self.processed_frames} frames - {fps:.1f} FPS")
                
                # 定期发布中间地图
                self.publish_intermediate_map()
            
        except Exception as e:
            rospy.logerr(f"Error processing frame: {e}")
            import traceback
            traceback.print_exc()
    
    def pointcloud2_to_numpy(self, cloud_msg):
        """将PointCloud2消息转换为numpy数组"""
        try:
            points_list = []
            for point in pc2.read_points(cloud_msg, skip_nans=True):
                # 支持不同的点云格式
                if len(point) >= 3:
                    points_list.append([point[0], point[1], point[2]])
            
            if len(points_list) == 0:
                return None
            
            return np.array(points_list, dtype=np.float32)
        except Exception as e:
            rospy.logerr(f"Error converting pointcloud: {e}")
            return None
    
    def transform_pointcloud(self, points, transform):
        """将点云变换到目标坐标系"""
        # 提取变换参数
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # 四元数转旋转矩阵
        R = self.quaternion_to_rotation_matrix(rot)
        t = np.array([trans.x, trans.y, trans.z])
        
        # 应用变换
        points_transformed = np.dot(points[:, :3], R.T) + t
        
        return points_transformed
    
    def quaternion_to_rotation_matrix(self, q):
        """四元数转旋转矩阵"""
        x, y, z, w = q.x, q.y, q.z, q.w
        
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        
        return R
    
    def filter_by_range(self, points):
        """范围过滤"""
        distances = np.linalg.norm(points[:, :3], axis=1)
        mask = (distances >= self.min_range) & (distances <= self.max_range)
        return points[mask]
    
    def segment_ground(self, points):
        """地面分割"""
        if len(points) < 100:
            return np.array([]), points
        
        # 转换为Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        # RANSAC平面拟合
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.ground_height_threshold,
            ransac_n=3,
            num_iterations=100
        )
        
        # 提取地面和非地面点
        ground_mask = np.zeros(len(points), dtype=bool)
        ground_mask[inliers] = True
        
        ground_points = points[ground_mask]
        non_ground_points = points[~ground_mask]
        
        return ground_points, non_ground_points
    
    def detect_dynamic_objects(self, points, ground_points):
        """综合动态物体检测"""
        if len(points) == 0:
            return np.array([]), np.array([])
        
        # 初始化掩码
        static_mask = np.ones(len(points), dtype=bool)
        dynamic_scores = np.zeros(len(points))
        
        # 方法1：基于历史帧的KNN检测
        if len(self.frame_buffer) > 0:
            historical_points = np.vstack(list(self.frame_buffer)[-5:])
            if len(historical_points) > 100:
                tree = KDTree(historical_points[:, :3])
                distances, _ = tree.query(points[:, :3], k=1)
                dynamic_scores += (distances > self.voxel_size * 2).astype(float) * 0.3
        
        # 方法2：基于范围图像的检测
        if self.use_range_image and len(self.frame_buffer) >= 2:
            current_range = self.range_image_processor.points_to_range_image(points)
            prev_range = self.range_image_processor.points_to_range_image(
                self.frame_buffer[-2])
            
            diff_mask = self.range_image_processor.compare_range_images(
                current_range, prev_range, threshold=1.0)
            
            # 将范围图像差异映射回点云
            for i, point in enumerate(points):
                px, py = self.point_to_range_pixel(point)
                if 0 <= px < diff_mask.shape[1] and 0 <= py < diff_mask.shape[0]:
                    if diff_mask[py, px]:
                        dynamic_scores[i] += 0.3
        
        # 方法3：基于体素占用历史
        for i, point in enumerate(points):
            voxel_key = self.voxel_grid.get_voxel_key(point)
            dynamic_ratio = self.voxel_grid.get_dynamic_ratio(voxel_key)
            occupancy_var = self.voxel_grid.get_occupancy_variance(voxel_key)
            
            dynamic_scores[i] += dynamic_ratio * 0.2
            dynamic_scores[i] += min(occupancy_var, 1.0) * 0.2
        
        # 方法4：基于聚类的动态检测
        if len(points) > 100:
            clustering = DBSCAN(
                eps=self.cluster_tolerance, 
                min_samples=self.min_cluster_size
            ).fit(points[:, :3])
            
            labels = clustering.labels_
            unique_labels = set(labels) - {-1}
            
            for label in unique_labels:
                cluster_mask = labels == label
                cluster_points = points[cluster_mask]
                
                # 检查簇的运动特性
                if self.is_cluster_dynamic(cluster_points):
                    dynamic_scores[cluster_mask] += 0.3
        
        # 综合判定
        dynamic_mask = dynamic_scores > self.dynamic_threshold
        static_mask = ~dynamic_mask
        
        return static_mask, dynamic_mask
    
    def point_to_range_pixel(self, point):
        """将3D点转换为范围图像像素坐标"""
        depth = np.linalg.norm(point[:3])
        yaw = np.arctan2(point[1], point[0])
        pitch = np.arcsin(point[2] / (depth + 1e-10))
        
        px = int((yaw + np.pi) / (2 * np.pi) * self.range_image_processor.width)
        py = int((1.0 - (pitch - self.range_image_processor.fov_down) / 
                  self.range_image_processor.fov_vertical) * self.range_image_processor.height)
        
        return px, py
    
    def is_cluster_dynamic(self, cluster_points):
        """判断点簇是否为动态"""
        if len(self.frame_buffer) < 2:
            return False
        
        # 计算簇中心
        center = np.mean(cluster_points[:, :3], axis=0)
        
        # 检查历史帧中是否存在类似的簇
        prev_points = self.frame_buffer[-2]
        if len(prev_points) > 0:
            tree = KDTree(prev_points[:, :3])
            distances, _ = tree.query([center], k=10)
            
            # 如果最近邻距离较大，可能是新出现的物体
            if np.mean(distances) > self.cluster_tolerance * 2:
                return True
        
        return False
    
    def update_voxel_grid(self, static_points, dynamic_points, timestamp):
        """更新体素网格统计"""
        # 更新静态点
        for point in static_points:
            self.voxel_grid.add_point(point, timestamp, is_dynamic=False)
        
        # 更新动态点
        for point in dynamic_points:
            self.voxel_grid.add_point(point, timestamp, is_dynamic=True)
    
    def publish_frame_results(self, static_points, dynamic_points, 
                             ground_points, timestamp):
        """发布当前帧的处理结果"""
        # 发布静态点云
        if len(static_points) > 0:
            self.publish_pointcloud(static_points, self.static_cloud_pub, 
                                   timestamp, color=[0, 255, 0])
        
        # 发布动态点云
        if len(dynamic_points) > 0:
            self.publish_pointcloud(dynamic_points, self.dynamic_cloud_pub, 
                                   timestamp, color=[255, 0, 0])
        
        # 发布地面点云
        if len(ground_points) > 0:
            self.publish_pointcloud(ground_points, self.ground_cloud_pub, 
                                   timestamp, color=[128, 128, 128])
    
    def publish_intermediate_map(self):
        """发布中间地图结果"""
        # 收集静态点
        static_points = []
        for voxel_key, voxel_data in self.voxel_grid.voxels.items():
            if voxel_data['count'] >= self.min_observations:
                dynamic_ratio = self.voxel_grid.get_dynamic_ratio(voxel_key)
                if dynamic_ratio < self.dynamic_threshold:
                    # 使用体素中心或平均点
                    if len(voxel_data['points']) > 0:
                        static_points.append(np.mean(voxel_data['points'], axis=0))
        
        if len(static_points) > 0:
            static_points = np.array(static_points)
            self.publish_pointcloud(static_points, self.map_cloud_pub, 
                                   rospy.Time.now())
    
    def generate_final_map(self):
        """生成最终的静态地图"""
        rospy.loginfo("Generating final static map...")
        
        # 收集所有静态体素的点
        static_points = []
        for voxel_key, voxel_data in self.voxel_grid.voxels.items():
            if voxel_data['count'] >= self.min_observations:
                dynamic_ratio = self.voxel_grid.get_dynamic_ratio(voxel_key)
                if dynamic_ratio < self.dynamic_threshold:
                    # 使用所有点或采样
                    points = np.array(voxel_data['points'])
                    if len(points) > 10:
                        # 随机采样以减少点数
                        indices = np.random.choice(len(points), min(10, len(points)), replace=False)
                        points = points[indices]
                    static_points.extend(points)
        
        if len(static_points) == 0:
            rospy.logwarn("No static points found!")
            return np.array([])
        
        static_points = np.array(static_points)
        rospy.loginfo(f"Collected {len(static_points)} static points")
        
        # 创建Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(static_points[:, :3])
        
        # 下采样
        rospy.loginfo("Downsampling...")
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # 统计滤波去除离群点
        if self.use_statistical_filter and len(pcd.points) > 0:
            rospy.loginfo("Removing outliers...")
            pcd, ind = pcd.remove_statistical_outlier(
                nb_neighbors=int(self.statistical_mean_k),
                std_ratio=self.statistical_std_dev
            )
        
        # 保存处理后的点云
        final_points = np.asarray(pcd.points)
        rospy.loginfo(f"Final static map: {len(final_points)} points")
        return final_points
    
    def save_results(self):
        """保存处理结果"""
        # 生成最终地图
        final_points = self.generate_final_map()
        if len(final_points) == 0:
            return False
        
        # 保存PCD文件
        pcd_file = rospy.get_param('~output_pcd', 'static_map.pcd')
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(final_points)
            o3d.io.write_point_cloud(pcd_file, pcd)
            rospy.loginfo(f"Saved PCD file: {pcd_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save PCD: {e}")
            return False
        
        # 保存二进制文件
        bin_file = rospy.get_param('~output_bin', 'static_map.bin')
        try:
            final_points.astype(np.float32).tofile(bin_file)
            rospy.loginfo(f"Saved binary file: {bin_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save binary: {e}")
            return False
        
        # 保存统计信息
        stats_file = rospy.get_param('~output_stats', 'processing_stats.yaml')
        try:
            stats = {
                'total_frames': self.processed_frames,
                'total_voxels': len(self.voxel_grid.voxels),
                'static_points': len(final_points),
                'parameters': {
                    'voxel_size': self.voxel_size,
                    'dynamic_threshold': self.dynamic_threshold,
                    'observation_window': self.observation_window
                }
            }
            with open(stats_file, 'w') as f:
                yaml.dump(stats, f)
            rospy.loginfo(f"Saved statistics: {stats_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save stats: {e}")
            return False
        
        return True
    
    def handle_save_map(self, req):
        """处理保存地图服务请求"""
        try:
            success = self.save_results()
            if success:
                return TriggerResponse(success=True, message="Static map saved successfully")
            else:
                return TriggerResponse(success=False, message="Failed to save static map")
        except Exception as e:
            rospy.logerr(f"Error in save map service: {e}")
            return TriggerResponse(success=False, message=f"Error: {str(e)}")
    
    def publish_pointcloud(self, points, publisher, timestamp, color=None):
        """发布点云到ROS话题"""
        if len(points) == 0:
            return
        
        # 创建PointCloud2消息
        header = Header()
        header.stamp = timestamp
        header.frame_id = self.map_frame
        
        # 如果有颜色信息，添加RGB字段
        if color is not None:
            # 创建带颜色的点云
            rgb = np.full((len(points), 3), color, dtype=np.uint8)
            rgb_packed = (rgb[:, 0].astype(np.uint32) << 16 | 
                         rgb[:, 1].astype(np.uint32) << 8 | 
                         rgb[:, 2].astype(np.uint32))
            rgb_float = rgb_packed.view(np.float32)
            
            # 合并XYZ和RGB
            points_rgb = np.column_stack([points[:, :3], rgb_float])
            
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1)
            ]
            cloud_msg = pc2.create_cloud(header, fields, points_rgb)
        else:
            # 创建普通点云
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            cloud_msg = pc2.create_cloud(header, fields, points[:, :3])
        
        # 发布
        publisher.publish(cloud_msg)


def main():
    """主函数"""
    rospy.init_node('dynamic_object_remover', anonymous=True)

    parser = argparse.ArgumentParser(
        description='Remove dynamic objects from 3D LiDAR point clouds and build static map',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  %(prog)s
  
  # Custom parameters
  %(prog)s --voxel_size 0.2 --dynamic_threshold 0.4
  
  # Use config file
  %(prog)s --config config.yaml
  
  # Enable fake TF
  %(prog)s --use_fake_tf true
        """
    )
    
    # 算法参数
    parser.add_argument('--voxel_size', type=float, 
                       help='Voxel size in meters')
    parser.add_argument('--dynamic_threshold', type=float, 
                       help='Dynamic threshold 0-1')
    parser.add_argument('--min_observations', type=int, 
                       help='Minimum observations for static')
    parser.add_argument('--observation_window', type=int, 
                       help='Observation window size')
    parser.add_argument('--max_range', type=float, 
                       help='Maximum range in meters')
    parser.add_argument('--min_range', type=float, 
                       help='Minimum range in meters')
    
    # 话题参数
    parser.add_argument('--pointcloud_topic', type=str, 
                       help='Point cloud topic name')
    parser.add_argument('--map_frame', type=str, 
                       help='Map frame name')
    parser.add_argument('--lidar_frame', type=str, 
                       help='LiDAR frame name')
    
    # TF参数
    parser.add_argument('--use_fake_tf', type=bool, 
                       help='Enable fake TF broadcaster')
    
    # 输出参数
    parser.add_argument('--output_pcd', type=str, 
                       help='Output PCD file')
    parser.add_argument('--output_bin', type=str, 
                       help='Output binary file')
    
    # 配置文件
    parser.add_argument('--config', type=str, 
                       help='Configuration YAML file')
    
    # 调试选项
    parser.add_argument('--verbose', action='store_true',
                       help='Enable verbose output')
    
    args, unknown = parser.parse_known_args()
    
    # 设置ROS参数
    for key, value in vars(args).items():
        if value is not None and key not in ['config', 'verbose']:
            rospy.set_param(f'~{key}', value)
    
    # 设置日志级别
    if args.verbose:
        import logging
        logging.basicConfig(level=logging.DEBUG)
    
    try:
        # 创建处理器
        remover = DynamicObjectRemover(config=args.config)
        
        rospy.loginfo("Node started. Waiting for pointcloud data...")
        rospy.loginfo("To save map: rosservice call /save_static_map \"{}\"")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
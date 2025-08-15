#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs
from geometry_msgs.msg import TransformStamped
import threading
import time
import os
from datetime import datetime
import pickle

class PointCloudMapBuilder:
    def __init__(self):
        rospy.init_node('pointcloud_map_builder')
        
        # 参数配置
        self.input_topic = rospy.get_param('~input_topic', '/filtered_pointcloud')
        self.output_topic = rospy.get_param('~output_topic', '/accumulated_map')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        
        # 拼接策略配置
        self.save_mode = rospy.get_param('~save_mode', 'time')  # 'time' 或 'frame'
        self.time_interval = rospy.get_param('~time_interval', 2.0)  # 时间间隔（秒）
        self.frame_interval = rospy.get_param('~frame_interval', 5)   # 帧数间隔
        
        # 地图管理配置
        self.max_map_points = rospy.get_param('~max_map_points', 1000000)  # 最大地图点数
        self.voxel_size = rospy.get_param('~voxel_size', 0.05)  # 体素滤波大小
        self.enable_voxel_filter = rospy.get_param('~enable_voxel_filter', True)
        
        # 地图保存配置
        self.auto_save = rospy.get_param('~auto_save', True)
        self.save_interval = rospy.get_param('~save_interval', 60.0)  # 自动保存间隔（秒）
        self.save_directory = rospy.get_param('~save_directory', '~/pointcloud_maps')
        
        # 地图质量配置
        self.min_points_threshold = rospy.get_param('~min_points_threshold', 100)  # 最小点数阈值
        self.max_distance = rospy.get_param('~max_distance', 50.0)  # 最大距离阈值
        
        # TF处理
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 数据存储
        self.accumulated_points = []  # 累积的地图点
        self.last_save_time = time.time()
        self.last_auto_save_time = time.time()
        self.frame_count = 0
        self.total_frames_added = 0
        
        # 线程锁
        self.map_lock = threading.Lock()
        
        # 统计信息
        self.total_input_points = 0
        self.total_map_points = 0
        self.frames_processed = 0
        self.frames_saved = 0
        
        # 创建保存目录
        self.save_directory = os.path.expanduser(self.save_directory)
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        
        # 创建订阅者和发布者
        self.pc_subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1
        )
        
        self.map_publisher = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=1,
            latch=True  # 持续发布最新地图
        )
        
        # 创建服务
        self.save_map_service = rospy.Service(
            '~save_map',
            Empty,
            self.save_map_callback
        )
        
        self.clear_map_service = rospy.Service(
            '~clear_map',
            Empty,
            self.clear_map_callback
        )
        
        # 发布地图的定时器
        self.map_publish_timer = rospy.Timer(
            rospy.Duration(1.0),  # 每秒发布一次地图
            self.publish_map_timer_callback
        )
        
        # 自动保存定时器
        if self.auto_save:
            self.auto_save_timer = rospy.Timer(
                rospy.Duration(self.save_interval),
                self.auto_save_timer_callback
            )
        
        rospy.loginfo("点云地图构建节点已启动")
        rospy.loginfo(f"输入话题: {self.input_topic}")
        rospy.loginfo(f"输出话题: {self.output_topic}")
        rospy.loginfo(f"地图坐标系: {self.map_frame}")
        rospy.loginfo(f"保存模式: {self.save_mode}")
        
        if self.save_mode == 'time':
            rospy.loginfo(f"时间间隔: {self.time_interval}秒")
        else:
            rospy.loginfo(f"帧数间隔: {self.frame_interval}帧")
        
        rospy.loginfo(f"体素滤波: {'启用' if self.enable_voxel_filter else '禁用'} (大小: {self.voxel_size}m)")
        rospy.loginfo(f"最大地图点数: {self.max_map_points}")
        rospy.loginfo(f"保存目录: {self.save_directory}")
    
    def pointcloud_callback(self, msg):
        """处理输入的点云数据"""
        current_time = time.time()
        self.frame_count += 1
        self.frames_processed += 1
        
        # 检查是否应该保存这一帧
        should_save = False
        
        if self.save_mode == 'time':
            if current_time - self.last_save_time >= self.time_interval:
                should_save = True
                self.last_save_time = current_time
        else:  # frame mode
            if self.frame_count >= self.frame_interval:
                should_save = True
                self.frame_count = 0
        
        if not should_save:
            return
        
        # 确保点云在正确的坐标系中
        if msg.header.frame_id != self.map_frame:
            transformed_msg = self.transform_pointcloud_to_map(msg)
            if transformed_msg is None:
                rospy.logwarn("点云坐标变换失败，跳过此帧")
                return
            msg = transformed_msg
        
        # 读取点云数据
        try:
            points_list = list(pc2.read_points(msg, skip_nans=True))
            if len(points_list) < self.min_points_threshold:
                rospy.logdebug(f"点云点数过少 ({len(points_list)} < {self.min_points_threshold})，跳过")
                return
            
            # 距离滤波
            filtered_points = self.distance_filter(points_list)
            if not filtered_points:
                rospy.logdebug("距离滤波后无有效点，跳过")
                return
            
            # 添加到累积地图
            self.add_points_to_map(filtered_points, msg.header)
            
            self.frames_saved += 1
            self.total_input_points += len(points_list)
            
            rospy.loginfo(f"添加第 {self.frames_saved} 帧到地图 - "
                         f"输入点数: {len(points_list)}, "
                         f"有效点数: {len(filtered_points)}, "
                         f"总地图点数: {len(self.accumulated_points)}")
            
        except Exception as e:
            rospy.logerr(f"处理点云时出错: {str(e)}")
    
    def transform_pointcloud_to_map(self, pointcloud_msg):
        """将点云转换到地图坐标系"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                pointcloud_msg.header.frame_id,
                pointcloud_msg.header.stamp,
                rospy.Duration(1.0)
            )
            
            transformed_pc = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)
            transformed_pc.header.frame_id = self.map_frame
            
            return transformed_pc
            
        except Exception as e:
            rospy.logwarn(f"TF变换失败: {str(e)}")
            return None
    
    def distance_filter(self, points_list):
        """距离滤波，移除过远的点"""
        filtered_points = []
        for point in points_list:
            x, y, z = point[:3]
            distance = np.sqrt(x*x + y*y + z*z)
            
            if distance <= self.max_distance:
                filtered_points.append(point)
        
        return filtered_points
    
    def voxel_filter(self, points):
        """简单的体素滤波实现"""
        if not self.enable_voxel_filter or not points:
            return points
        
        # 将点转换为numpy数组
        points_array = np.array([[p[0], p[1], p[2]] for p in points])
        
        # 计算体素网格索引
        voxel_indices = np.floor(points_array / self.voxel_size).astype(int)
        
        # 使用字典来存储每个体素中的点
        voxel_dict = {}
        for i, voxel_idx in enumerate(voxel_indices):
            key = tuple(voxel_idx)
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(points[i])
        
        # 每个体素取中心点
        filtered_points = []
        for voxel_points in voxel_dict.values():
            if len(voxel_points) == 1:
                filtered_points.append(voxel_points[0])
            else:
                # 计算体素内点的平均值
                avg_point = np.mean([[p[0], p[1], p[2]] for p in voxel_points], axis=0)
                # 保持原始点的其他属性（如果有的话）
                new_point = list(voxel_points[0])  # 复制第一个点的属性
                new_point[0] = avg_point[0]  # 更新x
                new_point[1] = avg_point[1]  # 更新y
                new_point[2] = avg_point[2]  # 更新z
                filtered_points.append(tuple(new_point))
        
        return filtered_points
    
    def add_points_to_map(self, points, header):
        """添加点到累积地图"""
        with self.map_lock:
            # 体素滤波
            filtered_points = self.voxel_filter(points)
            
            # 添加到累积点云
            self.accumulated_points.extend(filtered_points)
            
            # 控制地图大小
            if len(self.accumulated_points) > self.max_map_points:
                # 移除最旧的点（简单的FIFO策略）
                excess_points = len(self.accumulated_points) - self.max_map_points
                self.accumulated_points = self.accumulated_points[excess_points:]
                rospy.loginfo(f"地图点数达到上限，移除了 {excess_points} 个旧点")
            
            self.total_map_points = len(self.accumulated_points)
            self.total_frames_added += 1
    
    def publish_map_timer_callback(self, event):
        """定时发布累积地图"""
        with self.map_lock:
            if not self.accumulated_points:
                return
            
            try:
                # 创建地图点云消息
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = self.map_frame
                
                # 定义点云字段
                fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                ]
                
                # 创建点云消息
                map_msg = pc2.create_cloud(header, fields, self.accumulated_points)
                
                # 发布地图
                self.map_publisher.publish(map_msg)
                
                rospy.logdebug(f"发布地图，包含 {len(self.accumulated_points)} 个点")
                
            except Exception as e:
                rospy.logerr(f"发布地图时出错: {str(e)}")
    
    def auto_save_timer_callback(self, event):
        """自动保存地图"""
        if time.time() - self.last_auto_save_time >= self.save_interval:
            self.save_map_to_file("auto_save")
            self.last_auto_save_time = time.time()
    
    def save_map_callback(self, request):
        """手动保存地图服务回调"""
        try:
            filename = self.save_map_to_file("manual_save")
            rospy.loginfo(f"地图已手动保存到: {filename}")
            return EmptyResponse()
        except Exception as e:
            rospy.logerr(f"保存地图失败: {str(e)}")
            return EmptyResponse()
    
    def clear_map_callback(self, request):
        """清空地图服务回调"""
        try:
            with self.map_lock:
                self.accumulated_points.clear()
                self.total_map_points = 0
                self.total_frames_added = 0
            
            rospy.loginfo("地图已清空")
            return EmptyResponse()
        except Exception as e:
            rospy.logerr(f"清空地图失败: {str(e)}")
            return EmptyResponse()
    
    def save_map_to_file(self, save_type="manual"):
        """保存地图到文件"""
        with self.map_lock:
            if not self.accumulated_points:
                rospy.logwarn("地图为空，无法保存")
                return None
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{save_type}_map_{timestamp}.pkl"
            filepath = os.path.join(self.save_directory, filename)
            
            # 保存数据
            map_data = {
                'points': self.accumulated_points,
                'timestamp': timestamp,
                'total_points': len(self.accumulated_points),
                'frames_added': self.total_frames_added,
                'map_frame': self.map_frame,
                'voxel_size': self.voxel_size,
                'save_mode': self.save_mode,
                'statistics': {
                    'total_input_points': self.total_input_points,
                    'frames_processed': self.frames_processed,
                    'frames_saved': self.frames_saved
                }
            }
            
            with open(filepath, 'wb') as f:
                pickle.dump(map_data, f)
            
            # 同时保存为PCD格式
            pcd_filename = f"{save_type}_map_{timestamp}.pcd"
            pcd_filepath = os.path.join(self.save_directory, pcd_filename)
            self.save_as_pcd(pcd_filepath)
            
            rospy.loginfo(f"地图已保存: {filepath} (点数: {len(self.accumulated_points)})")
            rospy.loginfo(f"PCD格式已保存: {pcd_filepath}")
            
            return filepath
    
    def save_as_pcd(self, filepath):
        """保存为PCD格式"""
        try:
            with open(filepath, 'w') as f:
                # PCD文件头
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(self.accumulated_points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(self.accumulated_points)}\n")
                f.write("DATA ascii\n")
                
                # 点云数据
                for point in self.accumulated_points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
                    
        except Exception as e:
            rospy.logerr(f"保存PCD文件失败: {str(e)}")
    
    def print_statistics(self):
        """打印统计信息"""
        with self.map_lock:
            rospy.loginfo("=== 地图构建统计 ===")
            rospy.loginfo(f"总处理帧数: {self.frames_processed}")
            rospy.loginfo(f"保存到地图的帧数: {self.frames_saved}")
            rospy.loginfo(f"总输入点数: {self.total_input_points}")
            rospy.loginfo(f"当前地图点数: {len(self.accumulated_points)}")
            rospy.loginfo(f"保存目录: {self.save_directory}")
            rospy.loginfo("==================")
    
    def run(self):
        """运行节点"""
        rospy.loginfo("等待输入点云数据...")
        
        # 定期打印统计信息
        stat_timer = rospy.Timer(rospy.Duration(30.0), lambda event: self.print_statistics())
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("正在保存地图并退出...")
            self.save_map_to_file("shutdown_save")
            self.print_statistics()


def main():
    try:
        builder = PointCloudMapBuilder()
        builder.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行出错: {str(e)}")


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rospy
import numpy as np
import pickle
import os
import argparse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class MapUtils:
    """地图工具类，用于加载、分析和处理保存的点云地图"""
    
    def __init__(self):
        rospy.init_node('map_utils', anonymous=True)
        self.map_publisher = rospy.Publisher('/loaded_map', PointCloud2, queue_size=1, latch=True)
    
    def load_map(self, filepath):
        """加载保存的地图文件"""
        try:
            with open(filepath, 'rb') as f:
                map_data = pickle.load(f)
            
            print(f"成功加载地图: {filepath}")
            print(f"时间戳: {map_data.get('timestamp', 'unknown')}")
            print(f"点云数量: {map_data.get('total_points', 0)}")
            print(f"帧数: {map_data.get('frames_added', 0)}")
            print(f"坐标系: {map_data.get('map_frame', 'unknown')}")
            print(f"体素大小: {map_data.get('voxel_size', 'unknown')}")
            
            if 'statistics' in map_data:
                stats = map_data['statistics']
                print(f"输入点数: {stats.get('total_input_points', 0)}")
                print(f"处理帧数: {stats.get('frames_processed', 0)}")
                print(f"保存帧数: {stats.get('frames_saved', 0)}")
            
            return map_data
            
        except Exception as e:
            print(f"加载地图失败: {str(e)}")
            return None
    
    def analyze_map(self, map_data):
        """分析地图数据"""
        if not map_data or 'points' not in map_data:
            print("无效的地图数据")
            return
        
        points = np.array([[p[0], p[1], p[2]] for p in map_data['points']])
        
        print("\n=== 地图分析 ===")
        print(f"点云数量: {len(points)}")
        
        # 边界框分析
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        ranges = max_coords - min_coords
        
        print(f"X范围: {min_coords[0]:.2f} ~ {max_coords[0]:.2f} (跨度: {ranges[0]:.2f}m)")
        print(f"Y范围: {min_coords[1]:.2f} ~ {max_coords[1]:.2f} (跨度: {ranges[1]:.2f}m)")
        print(f"Z范围: {min_coords[2]:.2f} ~ {max_coords[2]:.2f} (跨度: {ranges[2]:.2f}m)")
        
        # 密度分析
        volume = ranges[0] * ranges[1] * ranges[2] if all(ranges > 0) else 0
        if volume > 0:
            density = len(points) / volume
            print(f"点云密度: {density:.2f} 点/立方米")
        
        # 距离分析
        distances = np.sqrt(np.sum(points**2, axis=1))
        print(f"距离统计: 最小={np.min(distances):.2f}m, "
              f"最大={np.max(distances):.2f}m, "
              f"平均={np.mean(distances):.2f}m")
        
        # 高程分析
        print(f"高程统计: 最低={min_coords[2]:.2f}m, "
              f"最高={max_coords[2]:.2f}m, "
              f"平均={np.mean(points[:, 2]):.2f}m")
    
    def publish_map(self, map_data, frame_id='map'):
        """发布加载的地图到ROS话题"""
        if not map_data or 'points' not in map_data:
            print("无效的地图数据，无法发布")
            return
        
        try:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = frame_id
            
            # 创建点云消息
            fields = [
                sensor_msgs.msg.PointField('x', 0, sensor_msgs.msg.PointField.FLOAT32, 1),
                sensor_msgs.msg.PointField('y', 4, sensor_msgs.msg.PointField.FLOAT32, 1),
                sensor_msgs.msg.PointField('z', 8, sensor_msgs.msg.PointField.FLOAT32, 1),
            ]
            
            map_msg = pc2.create_cloud(header, fields, map_data['points'])
            self.map_publisher.publish(map_msg)
            
            print(f"地图已发布到话题 /loaded_map (坐标系: {frame_id})")
            
        except Exception as e:
            print(f"发布地图失败: {str(e)}")
    
    def crop_map(self, map_data, x_range=None, y_range=None, z_range=None):
        """裁剪地图"""
        if not map_data or 'points' not in map_data:
            print("无效的地图数据")
            return None
        
        points = map_data['points']
        cropped_points = []
        
        for point in points:
            x, y, z = point[0], point[1], point[2]
            
            # 检查是否在指定范围内
            if x_range and not (x_range[0] <= x <= x_range[1]):
                continue
            if y_range and not (y_range[0] <= y <= y_range[1]):
                continue
            if z_range and not (z_range[0] <= z <= z_range[1]):
                continue
            
            cropped_points.append(point)
        
        # 创建新的地图数据
        cropped_map_data = map_data.copy()
        cropped_map_data['points'] = cropped_points
        cropped_map_data['total_points'] = len(cropped_points)
        cropped_map_data['cropped'] = True
        cropped_map_data['crop_ranges'] = {
            'x_range': x_range,
            'y_range': y_range,
            'z_range': z_range
        }
        
        print(f"地图裁剪完成: {len(points)} -> {len(cropped_points)} 点")
        return cropped_map_data
    
    def merge_maps(self, map_files):
        """合并多个地图文件"""
        merged_points = []
        total_frames = 0
        
        print(f"开始合并 {len(map_files)} 个地图文件...")
        
        for i, filepath in enumerate(map_files):
            map_data = self.load_map(filepath)
            if map_data and 'points' in map_data:
                merged_points.extend(map_data['points'])
                total_frames += map_data.get('frames_added', 0)
                print(f"合并进度: {i+1}/{len(map_files)} - 添加 {len(map_data['points'])} 点")
        
        # 创建合并后的地图数据
        merged_map_data = {
            'points': merged_points,
            'timestamp': rospy.Time.now().to_sec(),
            'total_points': len(merged_points),
            'frames_added': total_frames,
            'map_frame': 'map',
            'merged': True,
            'source_files': map_files
        }
        
        print(f"地图合并完成: 总点数 {len(merged_points)}")
        return merged_map_data
    
    def save_map(self, map_data, output_path):
        """保存地图数据"""
        try:
            with open(output_path, 'wb') as f:
                pickle.dump(map_data, f)
            print(f"地图已保存到: {output_path}")
        except Exception as e:
            print(f"保存地图失败: {str(e)}")
    
    def convert_to_pcd(self, map_data, output_path):
        """转换为PCD格式"""
        try:
            with open(output_path, 'w') as f:
                points = map_data['points']
                
                # PCD文件头
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                
                # 点云数据
                for point in points:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
            
            print(f"PCD文件已保存到: {output_path}")
            
        except Exception as e:
            print(f"保存PCD文件失败: {str(e)}")
    
    def downsample_map(self, map_data, factor=2):
        """下采样地图"""
        if not map_data or 'points' not in map_data:
            print("无效的地图数据")
            return None
        
        points = map_data['points']
        downsampled_points = points[::factor]  # 简单的均匀下采样
        
        # 创建下采样后的地图数据
        downsampled_map_data = map_data.copy()
        downsampled_map_data['points'] = downsampled_points
        downsampled_map_data['total_points'] = len(downsampled_points)
        downsampled_map_data['downsampled'] = True
        downsampled_map_data['downsample_factor'] = factor
        
        print(f"地图下采样完成: {len(points)} -> {len(downsampled_points)} 点 (因子: {factor})")
        return downsampled_map_data


def main():
    parser = argparse.ArgumentParser(description='点云地图工具')
    parser.add_argument('command', choices=['load', 'analyze', 'publish', 'crop', 'merge', 'convert', 'downsample'], 
                       help='执行的操作')
    parser.add_argument('--input', '-i', help='输入地图文件路径')
    parser.add_argument('--output', '-o', help='输出文件路径')
    parser.add_argument('--files', nargs='+', help='多个输入文件（用于合并）')
    parser.add_argument('--frame', default='map', help='发布时使用的坐标系')
    
    # 裁剪参数
    parser.add_argument('--x-min', type=float, help='X轴最小值')
    parser.add_argument('--x-max', type=float, help='X轴最大值')
    parser.add_argument('--y-min', type=float, help='Y轴最小值')
    parser.add_argument('--y-max', type=float, help='Y轴最大值')
    parser.add_argument('--z-min', type=float, help='Z轴最小值')
    parser.add_argument('--z-max', type=float, help='Z轴最大值')
    
    # 下采样参数
    parser.add_argument('--factor', type=int, default=2, help='下采样因子')
    
    args = parser.parse_args()
    
    utils = MapUtils()
    
    if args.command == 'load':
        if not args.input:
            print("错误: 需要指定输入文件 (--input)")
            return
        
        map_data = utils.load_map(args.input)
        if map_data:
            utils.analyze_map(map_data)
    
    elif args.command == 'analyze':
        if not args.input:
            print("错误: 需要指定输入文件 (--input)")
            return
        
        map_data = utils.load_map(args.input)
        if map_data:
            utils.analyze_map(map_data)
    
    elif args.command == 'publish':
        if not args.input:
            print("错误: 需要指定输入文件 (--input)")
            return
        
        map_data = utils.load_map(args.input)
        if map_data:
            utils.publish_map(map_data, args.frame)
            print("保持ROS节点运行以持续发布地图...")
            rospy.spin()
    
    elif args.command == 'crop':
        if not args.input:
            print("错误: 需要指定输入文件 (--input)")
            return
        
        map_data = utils.load_map(args.input)
        if map_data:
            x_range = [args.x_min, args.x_max] if args.x_min is not None and args.x_max is not None else None
            y_range = [args.y_min, args.y_max] if args.y_min is not None and args.y_max is not None else None
            z_range = [args.z_min, args.z_max] if args.z_min is not None and args.z_max is not None else None
            
            cropped_map = utils.crop_map(map_data, x_range, y_range, z_range)
            if cropped_map and args.output:
                utils.save_map(cropped_map, args.output)
    
    elif args.command == 'merge':
        if not args.files or len(args.files) < 2:
            print("错误: 需要指定至少2个输入文件 (--files)")
            return
        
        merged_map = utils.merge_maps(args.files)
        if merged_map and args.output:
            utils.save_map(merged_map, args.output)
    
    elif args.command == 'convert':
        if not args.input or not args.output:
            print("错误: 需要指定输入文件 (--input) 和输出文件 (--output)")
            return
        
        map_data = utils.load_map(args.input)
        if map_data:
            utils.convert_to_pcd(map_data, args.output)
    
    elif args.command == 'downsample':
        if not args.input:
            print("错误: 需要指定输入文件 (--input)")
            return
        
        map_data = utils.load_map(args.input)
        if map_data:
            downsampled_map = utils.downsample_map(map_data, args.factor)
            if downsampled_map and args.output:
                utils.save_map(downsampled_map, args.output)


if __name__ == '__main__':
    main()


# 使用示例脚本
"""
使用示例：

1. 分析地图:
python3 map_utils.py analyze --input /path/to/map.pkl

2. 发布地图到ROS:
python3 map_utils.py publish --input /path/to/map.pkl --frame map

3. 裁剪地图:
python3 map_utils.py crop --input /path/to/map.pkl --output /path/to/cropped_map.pkl --x-min -10 --x-max 10 --y-min -5 --y-max 5

4. 合并多个地图:
python3 map_utils.py merge --files map1.pkl map2.pkl map3.pkl --output merged_map.pkl

5. 转换为PCD格式:
python3 map_utils.py convert --input /path/to/map.pkl --output /path/to/map.pcd

6. 下采样地图:
python3 map_utils.py downsample --input /path/to/map.pkl --output /path/to/downsampled_map.pkl --factor 3
"""
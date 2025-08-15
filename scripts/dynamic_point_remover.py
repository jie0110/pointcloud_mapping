#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs
from geometry_msgs.msg import Point
import struct
import threading

class DynamicPointRemover:
    def __init__(self):
        rospy.init_node('dynamic_point_remover')
        
        # 参数配置
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/transformed_pointcloud')
        self.marker_topic = rospy.get_param('~marker_topic', '/motion_detector/visualization/clusters')
        self.output_topic = rospy.get_param('~output_topic', '/filtered_pointcloud')
        self.expansion_margin = rospy.get_param('~expansion_margin', 0.2)  # 边界框扩展边距
        self.z_expansion = rospy.get_param('~z_expansion', 0.5)  # Z轴额外扩展
        self.max_marker_age = rospy.get_param('~max_marker_age', 1.0)  # 最大marker有效时间
        
        # 坐标系设置
        self.source_frame = rospy.get_param('~source_frame', 'body')  # 点云坐标系
        self.target_frame = rospy.get_param('~target_frame', 'map')   # 目标坐标系（marker坐标系）
        self.transform_timeout = rospy.get_param('~transform_timeout', 1.0)  # TF变换超时时间
        
        # TF处理
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 数据存储
        self.current_markers = []
        self.current_pointcloud = None
        self.marker_lock = threading.Lock()
        self.pointcloud_lock = threading.Lock()
        
        # 创建订阅者和发布者
        self.pc_subscriber = rospy.Subscriber(
            self.pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1
        )
        
        self.marker_subscriber = rospy.Subscriber(
            self.marker_topic,
            MarkerArray,
            self.marker_callback,
            queue_size=1
        )
        
        self.filtered_pc_publisher = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=1
        )
        
        # 统计信息
        self.total_points_processed = 0
        self.total_points_removed = 0
        self.transform_success_count = 0
        self.transform_fail_count = 0
        
        rospy.loginfo("动态点云过滤节点已启动")
        rospy.loginfo(f"输入点云话题: {self.pointcloud_topic}")
        rospy.loginfo(f"输入marker话题: {self.marker_topic}")
        rospy.loginfo(f"输出点云话题: {self.output_topic}")
        rospy.loginfo(f"源坐标系: {self.source_frame}")
        rospy.loginfo(f"目标坐标系: {self.target_frame}")
        rospy.loginfo(f"边界框扩展边距: {self.expansion_margin}m")
        
    def transform_pointcloud_to_target(self, pointcloud_msg):
        """将点云从源坐标系转换到目标坐标系"""
        try:
            # 如果点云的frame_id不是源坐标系，先更新它
            if pointcloud_msg.header.frame_id != self.source_frame:
                rospy.logwarn_once(
                    f"点云frame_id ({pointcloud_msg.header.frame_id}) 与配置的源坐标系 ({self.source_frame}) 不匹配，使用点云的frame_id"
                )
                source_frame = pointcloud_msg.header.frame_id
            else:
                source_frame = self.source_frame
            
            # 获取变换关系
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                pointcloud_msg.header.stamp,
                rospy.Duration(self.transform_timeout)
            )
            
            # 执行点云坐标变换
            transformed_pc = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)
            
            # 更新frame_id为目标坐标系
            transformed_pc.header.frame_id = self.target_frame
            
            self.transform_success_count += 1
            rospy.logdebug(f"成功将点云从 {source_frame} 转换到 {self.target_frame}")
            
            return transformed_pc
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.transform_fail_count += 1
            rospy.logwarn_throttle(5.0, f"TF变换失败 ({source_frame} -> {self.target_frame}): {str(e)}")
            return None
        except Exception as e:
            self.transform_fail_count += 1
            rospy.logerr(f"点云变换出现未知错误: {str(e)}")
            return None
    
    def marker_callback(self, msg):
        """处理MarkerArray消息"""
        # rospy.loginfo("收到marker数据")
        with self.marker_lock:
            # 过滤出边界框类型的marker (type=5是LINE_LIST，用于显示边界框)
            box_markers = []
            for marker in msg.markers:
                if marker.type == Marker.LINE_LIST and len(marker.points) > 0:
                    # 检查marker的坐标系
                    if marker.header.frame_id != self.target_frame:
                        rospy.logwarn_once(
                            f"Marker frame_id ({marker.header.frame_id}) 与目标坐标系 ({self.target_frame}) 不匹配"
                        )
                    box_markers.append(marker)
            
            self.current_markers = box_markers
            
        rospy.logdebug(f"收到 {len(box_markers)} 个边界框marker")
        
        # 如果有点云数据，立即进行过滤
        if self.current_pointcloud is not None:
            self.process_pointcloud()
    
    def pointcloud_callback(self, msg):
        """处理点云消息"""
        # rospy.loginfo("收到点云数据")
        # 首先将点云转换到目标坐标系（map）
        transformed_msg = self.transform_pointcloud_to_target(msg)
        if transformed_msg is None:
            rospy.logwarn("点云坐标变换失败，跳过此帧")
            return
        
        with self.pointcloud_lock:
            self.current_pointcloud = transformed_msg
        
        # 立即进行处理
        self.process_pointcloud()
    
    def extract_bounding_box_from_marker(self, marker):
        """从marker的点列表中提取边界框"""
        if len(marker.points) < 8:  # 边界框至少需要8个点
            return None
        
        # 提取所有点的坐标
        points = np.array([[p.x, p.y, p.z] for p in marker.points])
        
        # 计算边界框的最小最大值
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        
        # 应用扩展边距
        min_coords[0] -= self.expansion_margin  # x
        min_coords[1] -= self.expansion_margin  # y
        min_coords[2] -= self.z_expansion       # z额外扩展
        
        max_coords[0] += self.expansion_margin  # x
        max_coords[1] += self.expansion_margin  # y
        max_coords[2] += self.expansion_margin  # z
        
        return {
            'min_x': min_coords[0], 'max_x': max_coords[0],
            'min_y': min_coords[1], 'max_y': max_coords[1],
            'min_z': min_coords[2], 'max_z': max_coords[2]
        }
    
    def is_point_in_bounding_boxes(self, x, y, z, bounding_boxes):
        """检查点是否在任一边界框内"""
        for bbox in bounding_boxes:
            if (bbox['min_x'] <= x <= bbox['max_x'] and
                bbox['min_y'] <= y <= bbox['max_y'] and
                bbox['min_z'] <= z <= bbox['max_z']):
                return True
        return False
    
    def process_pointcloud(self):
        """处理点云数据，移除动态点"""
        with self.pointcloud_lock:
            if self.current_pointcloud is None:
                return
            pointcloud_msg = self.current_pointcloud
        
        with self.marker_lock:
            markers = self.current_markers.copy()
        
        if not markers:
            # 如果没有动态物体，直接发布转换后的点云
            self.filtered_pc_publisher.publish(pointcloud_msg)
            return
        
        # 提取边界框信息
        bounding_boxes = []
        for marker in markers:
            bbox = self.extract_bounding_box_from_marker(marker)
            if bbox is not None:
                bounding_boxes.append(bbox)
        
        if not bounding_boxes:
            # 如果没有有效的边界框，直接发布转换后的点云
            self.filtered_pc_publisher.publish(pointcloud_msg)
            return
        
        # 读取点云数据
        try:
            points_list = list(pc2.read_points(pointcloud_msg, skip_nans=True))
            if not points_list:
                rospy.logwarn("点云数据为空")
                return
            
            # 过滤点云
            filtered_points = []
            removed_count = 0
            
            for point in points_list:
                x, y, z = point[:3]  # 取前3个坐标
                
                # 检查点是否在边界框内
                if not self.is_point_in_bounding_boxes(x, y, z, bounding_boxes):
                    filtered_points.append(point)
                else:
                    removed_count += 1
            
            # 更新统计信息
            self.total_points_processed += len(points_list)
            self.total_points_removed += removed_count
            
            # 创建过滤后的点云消息
            filtered_msg = self.create_pointcloud_msg(filtered_points, pointcloud_msg)
            
            # 发布过滤后的点云（已经在map坐标系下）
            self.filtered_pc_publisher.publish(filtered_msg)
            
            # 定期输出统计信息
            rospy.loginfo_throttle(
                5.0,
                f"过滤统计 - 处理: {self.total_points_processed}, "
                f"移除: {self.total_points_removed}, "
                f"当前移除: {removed_count}/{len(points_list)}, "
                f"边界框: {len(bounding_boxes)}, "
                f"TF成功/失败: {self.transform_success_count}/{self.transform_fail_count}"
            )
            
        except Exception as e:
            rospy.logerr(f"点云处理错误: {str(e)}")
    
    def create_pointcloud_msg(self, points, original_msg):
        """创建PointCloud2消息"""
        if not points:
            # 如果没有点，创建空的点云
            filtered_msg = PointCloud2()
            filtered_msg.header = original_msg.header
            filtered_msg.height = 1
            filtered_msg.width = 0
            filtered_msg.fields = original_msg.fields
            filtered_msg.is_bigendian = original_msg.is_bigendian
            filtered_msg.point_step = original_msg.point_step
            filtered_msg.row_step = 0
            filtered_msg.data = b''
            filtered_msg.is_dense = original_msg.is_dense
            return filtered_msg
        
        # 创建包含过滤点的点云
        # 保持原始的字段结构
        fields = original_msg.fields
        
        # 将点数据转换为适当的格式
        filtered_msg = pc2.create_cloud(original_msg.header, fields, points)
        
        return filtered_msg
    
    def run(self):
        """运行节点"""
        rospy.loginfo("等待TF变换可用...")
        
        # 等待TF变换可用
        try:
            self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rospy.Time(0),
                rospy.Duration(10.0)
            )
            rospy.loginfo(f"TF变换已就绪: {self.source_frame} -> {self.target_frame}")
        except Exception as e:
            rospy.logwarn(f"等待TF变换超时: {str(e)}")
            rospy.loginfo("将继续运行，等待变换可用...")
        
        rospy.loginfo("等待点云和marker数据...")
        rospy.spin()


def main():
    try:
        remover = DynamicPointRemover()
        remover.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行出错: {str(e)}")


if __name__ == '__main__':
    main()

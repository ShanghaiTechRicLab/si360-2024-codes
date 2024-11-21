#!/usr/bin/env python3

import argparse
import math
import os
import shutil
import signal
import sys
import numpy as np
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

ros2_distro = ""

class KittiToRosbag(Node):
    def __init__(self, kitti_path, bag_path):
        super().__init__('kitti2rosbag')

        self.kitti_path = kitti_path
        self.bag_path = bag_path

        # 检查输入路径
        if not os.path.exists(kitti_path):
            self.get_logger().error(f"Kitti dataset not found at {kitti_path}")
            sys.exit(1)

        if os.path.exists(bag_path):
            self.get_logger().warn(f"Output bag file already exists at {bag_path}")
            shutil.rmtree(bag_path)
        
        # 设置 ROS2 bag 写入器
        storage_options = StorageOptions(
            uri=bag_path,
            storage_id='mcap'
        )
        converter_options = ConverterOptions('','')
        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        # 创建topic Metadata
        if ros2_distro == 'jazzy':
            topic_info = TopicMetadata(
                id=0,
                name='/velodyne_points',
                type='sensor_msgs/PointCloud2',
                serialization_format='cdr'
            )
        else:
            topic_info = TopicMetadata(
                name='/velodyne_points',
                type='sensor_msgs/PointCloud2',
                serialization_format='cdr'
            )
        self.writer.create_topic(topic_info)

    def write_pointcloud(self, timestamp, bin_path, topic):
        # 读取点云数据
        scan = np.fromfile(bin_path, dtype=np.float32)
        scan = scan.reshape((-1, 4))

        self.get_logger().info(f"Read {scan.shape[0]} points from {bin_path}")
        
        # 转换为ROS消息
        header = Header()
        header.stamp.sec = int(timestamp)
        header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        header.frame_id = "velodyne"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        pc_msg = point_cloud2.create_cloud(header, fields, scan)
        
        # 写入bag
        self.writer.write(
            topic,
            serialize_message(pc_msg),
            math.ceil(timestamp * 1e9)
        )

    def convert(self):
        # 读取时间戳文件
        timestamps = []
        with open(os.path.join(self.kitti_path, 'times.txt'), 'r') as f:
            timestamps = [float(line.strip()) for line in f]
            self.get_logger().info(f"Read {len(timestamps)} timestamps from {os.path.join(self.kitti_path, 'times.txt')}")

        for idx, timestamp in enumerate(timestamps):
            # 点云路径
            velodyne_path = os.path.join(self.kitti_path, 'velodyne', f'{idx:06d}.bin')

            # 写入数据
            if os.path.exists(velodyne_path):
                self.get_logger().info(f"Writing pointcloud {idx} to bag")
                self.write_pointcloud(timestamp, velodyne_path, '/velodyne_points')
        self.get_logger().info("Conversion completed")
        self.close()

    def close(self):
        sys.exit(0)

def parse_args():
    parser = argparse.ArgumentParser(description='Convert Kitti dataset to ros2 bag')
    
    parser.add_argument('kitti_path', type=str, help='Path to the Kitti dataset')
    parser.add_argument('bag_path', type=str, help='Path to the output bag file')
    return parser.parse_args()


def get_ros2_version():
    ros2_distro = os.environ.get('ROS_DISTRO')

    if ros2_distro not in ['humble', 'jazzy']:
        raise ValueError(f"Unsupported ROS2 distro: {ros2_distro}")
    
    return ros2_distro


def main():
    args = parse_args()

    ros2_distro = get_ros2_version()

    try:
        rclpy.init()
        rclpy.logging.get_logger('kitti2rosbag').info(f"ROS2 distro: {ros2_distro}")

        
        converter = KittiToRosbag(args.kitti_path, args.bag_path)
        signal.signal(signal.SIGINT, converter.close)
        signal.signal(signal.SIGTERM, converter.close)

        converter.convert()
        
        rclpy.shutdown()
    except Exception as e:
        rclpy.logging.get_logger('kitti2rosbag').error(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
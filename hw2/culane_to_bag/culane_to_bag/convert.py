#!/usr/bin/env python3

import argparse  # 导入命令行参数解析库
import os  # 导入操作系统相关的库
import cv2  # 导入OpenCV库，用于图像处理
import time  # 导入时间库
import rclpy  # 导入ROS2 Python库
import shutil  # 导入文件操作库
import math  # 导入数学库
from rclpy.node import Node  # 导入ROS2节点类
from rclpy.serialization import serialize_message  # 导入消息序列化函数
from cv_bridge import CvBridge  # 导入OpenCV与ROS图像转换库
from sensor_msgs.msg import Image  # 导入ROS图像消息类型
from lane_detection_msgs.msg import DetectionTarget, Lane  # 导入车道检测消息类型
import rosbag2_py  # 导入ROS2包库

class CulaneToBag(Node):
    def __init__(self, input_dir, output_bag):
        super().__init__('culane_to_bag')  # 初始化节点
        self.get_logger().info('Culane to Bag node has been started')  # 日志记录节点启动信息
        
        self.input_dir = input_dir  # 输入目录
        self.output_bag = output_bag  # 输出包文件
        self.cv_bridge = CvBridge()  # 初始化OpenCV与ROS图像转换桥接

        # 检查输出包文件是否存在，如果存在则删除
        if os.path.exists(self.output_bag):
            self.get_logger().info(f'Output bag file {self.output_bag} already exists, removing...')
            shutil.rmtree(self.output_bag)  # 删除输出包文件
        
        self.writer = rosbag2_py.SequentialWriter()  # 创建ROS2包写入器
        storage_options = rosbag2_py.StorageOptions(uri=self.output_bag, storage_id='sqlite3')  # 设置存储选项
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                        output_serialization_format='cdr')  # 设置转换选项
        self.writer.open(storage_options, converter_options)  # 打开包写入器
        
        # 创建检测目标主题
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(name='/detection_target',
                                     id=0,
                                     type='lane_detection_msgs/msg/DetectionTarget',
                                     serialization_format='cdr')
        )
        
        self.process_dataset()  # 处理数据集

    def process_dataset(self):
        frame_id = 0  # 初始化帧ID
        # 遍历输入目录中的所有文件
        for root, _, files in os.walk(self.input_dir):
            for file in files:
                if file.endswith('.jpg'):  # 只处理jpg文件
                    image_path = os.path.join(root, file)  # 获取图像路径
                    gt_path = os.path.join(root, file.replace('.jpg', '.lines.txt'))  # 获取对应的GT文件路径
                    
                    if os.path.exists(gt_path):  # 检查GT文件是否存在
                        frame_id += 1  # 增加帧ID
                        self.process_sample(image_path, gt_path, frame_id)  # 处理样本
                        time.sleep(1)  # 暂停1秒
        
        self.get_logger().info(f'Processed {frame_id} samples')  # 日志记录处理的样本数量

    def process_sample(self, image_path, gt_path, frame_id):
        # 读取图像
        cv_image = cv2.imread(image_path)  # 使用OpenCV读取图像
        ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")  # 转换为ROS图像消息
        
        # 读取GT车道
        gt_lanes = []  # 初始化GT车道列表
        with open(gt_path, 'r') as f:  # 打开GT文件
            for line in f:  # 遍历每一行
                points = line.strip().split()  # 分割点
                lane = Lane()  # 创建车道对象
                # print(points)  # 可选：打印点
                # lane.coordinates = [math.ceil(float(p)) for p in points]  # 可选：直接使用点
                for i in range(0, len(points), 2):  # 每两个点为一组
                    if math.ceil(float(points[i])) < 0 or math.ceil(float(points[i + 1])) < 0:  # 检查点是否有效
                        continue
                    lane.coordinates.append(math.ceil(float(points[i])))  # 添加x坐标
                    lane.coordinates.append(math.ceil(float(points[i + 1])))  # 添加y坐标
                gt_lanes.append(lane)  # 添加车道到GT列表
        
        # 创建DetectionTarget消息
        msg = DetectionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        msg.frame_id = frame_id  # 设置帧ID
        msg.image_raw = ros_image  # 设置原始图像
        msg.gt_lanes = gt_lanes  # 设置GT车道
        
        # 写入消息到包
        self.writer.write(
            '/detection_target',
            serialize_message(msg),  # 序列化消息
            self.get_clock().now().nanoseconds  # 获取当前时间的纳秒数
        )
        
        if frame_id % 100 == 0:  # 每处理100帧记录一次日志
            self.get_logger().info(f'Processed frame {frame_id}')

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2
    
    parser = argparse.ArgumentParser(description='Convert CuLane dataset to ROS2 bag')  # 创建参数解析器
    parser.add_argument('input_dir', help='Input directory containing CuLane dataset', type=str)  # 输入目录参数
    parser.add_argument('output_bag', help='Output bag file', type=str)  # 输出包文件参数
    args = parser.parse_args()  # 解析参数
    
    node = CulaneToBag(args.input_dir, args.output_bag)  # 创建CulaneToBag节点
    rclpy.spin(node)  # 运行节点
    node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':
    main()  # 运行主函数
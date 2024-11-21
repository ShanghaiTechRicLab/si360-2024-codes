#!/usr/bin/env python3

import argparse
import math
import os
import shutil
import signal
import sys
import time
import numpy as np
from datetime import datetime

import cv2
import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import scipy.spatial.transform as tf

from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path


ros2_distro = ""
MAX_RETRIES = 3


class KittiToRosbag(Node):
    def __init__(self, kitti_path, bag_path, calib_path, gt_pose_path):
        super().__init__("kitti2rosbag")

        self.kitti_path = kitti_path
        self.bag_path = bag_path
        self.bridge = CvBridge()
        self.gt_pose_path = gt_pose_path
        # 检查输入路径
        if not os.path.exists(kitti_path):
            self.get_logger().error(f"Kitti dataset not found at {kitti_path}")
            sys.exit(1)

        if os.path.exists(bag_path):
            self.get_logger().warn(f"Output bag file already exists at {bag_path}, remove it")
            shutil.rmtree(bag_path)

        if not os.path.exists(calib_path):
            self.get_logger().error(f"Calibration file {calib_path} does not exists!")
            sys.exit(1)
        if not os.path.exists(gt_pose_path):
            self.get_logger().error(
                f"Ground truth pose file {gt_pose_path} does not exists!"
            )
            sys.exit(1)

        # 设置 ROS2 bag 写入器
        storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
        converter_options = ConverterOptions("", "")
        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        # 创建topic Metadata
        self._create_topic_metadata()
        self.calib_data = self._read_calib(calib_path)
        self.camera_id_topic_mapping = {
            "0": "/camera/mono/left/",
            "1": "/camera/mono/right/",
            "2": "/camera/color/left/",
            "3": "/camera/color/right/",
        }
        self.camera_id_frame_id_mapping = {
            "0": "camera_mono_left",
            "1": "camera_mono_right",
            "2": "camera_color_left",
            "3": "camera_color_right",
        }

        self.camera_id_key_mapping = {"0": "P0", "1": "P1", "2": "P2", "3": "P3"}

    def _read_calib(self, calib_path):
        data = {}
        with open(calib_path, "r") as f:
            for line in f.readlines():
                key, value = line.split(":", 1)
                try:
                    data[key] = np.array([float(x) for x in value.split()]).reshape(
                        3, 4
                    )
                except ValueError:
                    pass
        assert len(data) in [
            4,
            5,
        ], "Invalid calibration file, it should contain 4 or 5 calibration matrices"
        return data

    def _read_pose(self, gt_pose_path):
        poses = []
        with open(gt_pose_path, "r") as f:
            for line in f.readlines():
                T = np.array([float(x) for x in line.split()]).reshape(3, 4)

                T_mat = np.identity(4)
                T_mat[:3, :] = T
                poses.append(T_mat)
        return poses

    def _create_topic_metadata(self):
        topics = [
            ("/velodyne_points", "sensor_msgs/PointCloud2", "cdr"),
            ("/camera/color/left/image_raw", "sensor_msgs/Image", "cdr"),
            ("/camera/color/left/camera_info", "sensor_msgs/CameraInfo", "cdr"),
            ("/camera/color/right/image_raw", "sensor_msgs/Image", "cdr"),
            ("/camera/color/right/camera_info", "sensor_msgs/CameraInfo", "cdr"),
            ("/tf_static", "tf2_msgs/TFMessage", "cdr"),
            ("/ground_truth/path", "nav_msgs/Path", "cdr"),
            ("/ground_truth/pose", "geometry_msgs/PoseStamped", "cdr"),
        ]

        id = 0
        for topic, type, serialization_format in topics:
            tf_static_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            offered_qos_profiles = "{durability: TRANSIENT_LOCAL, reliability: RELIABLE, history: KEEP_LAST, depth: 1}"
            if ros2_distro == "jazzy":
                if topic == "/tf_static":
                    self.writer.create_topic(
                        TopicMetadata(
                            id=id,
                            name=topic,
                            type=type,
                            serialization_format=serialization_format,
                            qos_profile=tf_static_qos,
                        )
                    )
                else:
                    self.writer.create_topic(
                        TopicMetadata(
                            id=id,
                            name=topic,
                            type=type,
                            serialization_format=serialization_format,
                        )
                    )
            elif ros2_distro == "humble":
                if topic == "/tf_static":
                    self.writer.create_topic(
                        TopicMetadata(
                            name=topic,
                            type=type,
                            serialization_format=serialization_format,
                            offered_qos_profiles=offered_qos_profiles,
                        )
                    )
                else:
                    self.writer.create_topic(
                        TopicMetadata(
                            name=topic,
                            type=type,
                            serialization_format=serialization_format,
                        )
                    )
            else:
                raise ValueError(
                    f"Unsupported ROS2 distro: {ros2_distro}, please use humble or jazzy"
                )
            id += 1

    def _create_static_transform(self):
        Tr = self.calib_data.get("Tr", None)
        if Tr is None:
            self.get_logger().error("Tr not found in calibration file")
            return

        tf_msg = TFMessage()

        camera_left_transform = TransformStamped()
        camera_left_transform.header.frame_id = "base_link"
        camera_left_transform.child_frame_id = "camera_color_left"
        camera_left_transform.transform.translation.x = 0.0
        camera_left_transform.transform.translation.y = 0.0
        camera_left_transform.transform.translation.z = 0.0
        camera_left_transform.transform.rotation.x = 0.0
        camera_left_transform.transform.rotation.y = 0.0
        camera_left_transform.transform.rotation.z = 0.0
        camera_left_transform.transform.rotation.w = 1.0

        velodyne_transform = TransformStamped()
        velodyne_transform.header.frame_id = "base_link"
        velodyne_transform.child_frame_id = "velodyne"
        velodyne_transform.transform.translation.x = Tr[0, 3]
        velodyne_transform.transform.translation.y = Tr[1, 3]
        velodyne_transform.transform.translation.z = Tr[2, 3]
        R = Tr[:3, :3]
        quat = tf.Rotation.from_matrix(R).as_quat()
        velodyne_transform.transform.rotation.x = quat[0]
        velodyne_transform.transform.rotation.y = quat[1]
        velodyne_transform.transform.rotation.z = quat[2]
        velodyne_transform.transform.rotation.w = quat[3]

        tf_msg.transforms.append(camera_left_transform)
        tf_msg.transforms.append(velodyne_transform)

        return tf_msg

    def _create_pose_message(self, pose_mat, timestamp):
        """创建 PoseStamped 消息"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp.sec = int(timestamp)
        pose_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        pose_msg.header.frame_id = "map"  # 或其他合适的 frame

        # 从变换矩阵中提取位置
        pose_msg.pose.position.x = pose_mat[0, 3]
        pose_msg.pose.position.y = pose_mat[1, 3]
        pose_msg.pose.position.z = pose_mat[2, 3]

        # 从旋转矩阵转换为四元数
        from scipy.spatial.transform import Rotation

        R = Rotation.from_matrix(pose_mat[:3, :3])
        quat = R.as_quat()  # [x, y, z, w] 格式

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        return pose_msg

    def _create_path_message(self, poses, timestamp):
        """创建累积路径消息"""
        path_msg = Path()
        path_msg.header.stamp.sec = int(timestamp)  
        path_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        path_msg.header.frame_id = "map"

        for pose_mat in poses:
            pose_stamped = self._create_pose_message(pose_mat, timestamp)
            path_msg.poses.append(pose_stamped)

        return path_msg

    def _write_path_message(self, path_msg, timestamp):
        self.writer.write(
            "/ground_truth/path",
            serialize_message(path_msg),
            math.ceil(timestamp * 1e9),
        )

    def _write_pose_message(self, pose_msg, timestamp):
        self.writer.write(
            "/ground_truth/pose",
            serialize_message(pose_msg),
            math.ceil(timestamp * 1e9),
        )

    def _write_pointcloud(self, timestamp, bin_path, topic):
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
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        pc_msg = point_cloud2.create_cloud(header, fields, scan)

        # 写入bag
        self.writer.write(topic, serialize_message(pc_msg), math.ceil(timestamp * 1e9))

    def _write_image(self, timestamp, image_path, camera_id):
        img = None
        for i in range(MAX_RETRIES):
            img = cv2.imread(image_path)
            if img is not None:
                break
            self.get_logger().warn(f"Failed to read image from {image_path}, retry {i}")
            time.sleep(0.1)
        if img is None:
            self.get_logger().error(f"Failed to read image from {image_path}")
            return
        if camera_id in ["2", "3"]:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.encoding = "bgr8"
        else:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
            img_msg.encoding = "mono8"
        img_msg.header.stamp.sec = int(timestamp)
        img_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        img_msg.header.frame_id = self.camera_id_frame_id_mapping[camera_id]

        image_topic = self.camera_id_topic_mapping[camera_id] + "image_raw"

        self.writer.write(
            image_topic, serialize_message(img_msg), math.ceil(timestamp * 1e9)
        )

        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp.sec = int(timestamp)
        camera_info_msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        camera_info_msg.header.frame_id = self.camera_id_frame_id_mapping[camera_id]
        camera_info_msg.width = 1242
        camera_info_msg.height = 375
        camera_info_idx = self.camera_id_key_mapping[camera_id]
        P = self.calib_data[camera_info_idx]
        camera_info_msg.k = [
            P[0, 0],
            P[0, 1],
            P[0, 2],
            P[1, 0],
            P[1, 1],
            P[1, 2],
            P[2, 0],
            P[2, 1],
            P[2, 2],
        ]
        camera_info_msg.p = [
            P[0, 0],
            P[0, 1],
            P[0, 2],
            P[0, 3],
            P[1, 0],
            P[1, 1],
            P[1, 2],
            P[1, 3],
            P[2, 0],
            P[2, 1],
            P[2, 2],
            P[2, 3],
        ]
        camera_info_topic = self.camera_id_topic_mapping[camera_id] + "camera_info"
        self.writer.write(
            camera_info_topic,
            serialize_message(camera_info_msg),
            math.ceil(timestamp * 1e9),
        )

    def convert(self):
        # 读取时间戳文件
        timestamps = []
        self._create_topic_metadata()
        with open(os.path.join(self.kitti_path, "times.txt"), "r") as f:
            timestamps = [float(line.strip()) for line in f]
            self.get_logger().info(
                f"Read {len(timestamps)} timestamps from {os.path.join(self.kitti_path, 'times.txt')}"
            )

        tf_msg = self._create_static_transform()
        self.writer.write(
            "/tf_static",
            serialize_message(tf_msg),
            math.ceil(timestamps[0] * 1e9),
        )

        poses = self._read_pose(self.gt_pose_path)

        for idx, timestamp in enumerate(timestamps):
            # 点云路径
            velodyne_path = os.path.join(self.kitti_path, "velodyne", f"{idx:06d}.bin")

            # 写入数据
            if os.path.exists(velodyne_path):
                self.get_logger().info(
                    f"Writing pointcloud {idx} to bag:{velodyne_path}"
                )
                self._write_pointcloud(timestamp, velodyne_path, "/velodyne_points")

            # 写入图像数据
            for camera_id in ["2", "3"]:
                image_path = os.path.join(
                    self.kitti_path, f"image_{camera_id}", f"{idx:06d}.png"
                )
                if os.path.exists(image_path):
                    self.get_logger().info(
                        f"Writing image {idx}:{camera_id} to bag:{image_path}"
                    )
                    self._write_image(timestamp, image_path, camera_id)
                else:
                    self.get_logger().warn(f"Image {image_path} does not exist")

            # 写入位姿数据
            pose_mat = poses[idx]
            pose_msg = self._create_pose_message(pose_mat, timestamp)
            self._write_pose_message(pose_msg, timestamp)

            # 写入路径数据
            path_msg = self._create_path_message(poses[: idx + 1], timestamp)
            self._write_path_message(path_msg, timestamp)

        self.get_logger().info("Conversion completed")
        self.close()

    def close(self):
        sys.exit(0)


def parse_args():
    parser = argparse.ArgumentParser(description="Convert Kitti dataset to ros2 bag")

    parser.add_argument("kitti_path", type=str, help="Path to the Kitti dataset")
    parser.add_argument("calib_path", type=str, help="Path to the calibration file")
    parser.add_argument(
        "gt_pose_path",
        type=str,
        help="Path to the ground truth pose file",
        default="",
    )
    parser.add_argument(
        "bag_path",
        type=str,
        help="Path to the output bag file",
    )

    return parser.parse_args()


def get_ros2_version():
    ros2_distro = os.environ.get("ROS_DISTRO")

    if ros2_distro not in ["humble", "jazzy"]:
        raise ValueError(f"Unsupported ROS2 distro: {ros2_distro}")

    return ros2_distro


def main():
    global ros2_distro
    args = parse_args()

    ros2_distro = get_ros2_version()

    try:
        rclpy.init()
        rclpy.logging.get_logger("kitti2rosbag").info(f"ROS2 distro: {ros2_distro}")

        converter = KittiToRosbag(
            args.kitti_path, args.bag_path, args.calib_path, args.gt_pose_path
        )
        signal.signal(signal.SIGINT, converter.close)
        signal.signal(signal.SIGTERM, converter.close)

        converter.convert()

        rclpy.shutdown()
    except Exception as e:
        rclpy.logging.get_logger("kitti2rosbag").error(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

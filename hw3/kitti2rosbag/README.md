# KITTI2rosbag 一个将 KITTI 数据集转化为 ros2 bag 的工具库


## 安装必须的包 

mcap 是 ros2 的日志格式，需要安装 mcap 的存储插件
```bash
sudo apt install -y ros-<ros_distro>-rosbag2-storage-mcap
```

## 使用

### 只转换 LiDAR 数据

```bash
ros2 run kitti2rosbag lidar_converter <kitti_path> <bag_path>
```

其中: 
- `<kitti_path>` 是 KITTI 数据集的路径，大部分情况下是这样的 `/xx/xx/dataset/kitti/dataset/sequences/00`
- `<bag_path>` 是输出的 bag 文件的路径, 例如 `bags/kitti_00_lidar.bag`

输出的数据将会在 `/velodyne_points` 话题下


### 转换所有数据

```bash
ros2 run kitti2rosbag all_converter <kitti_path> <calib_path> <gt_pose_path> <bag_path>
```

其中: 
- `<kitti_path>` 是 KITTI 数据集的路径，大部分情况下是这样的 `/xx/xx/dataset/kitti/dataset/sequences/00`
- `<calib_path>` 是 KITTI 数据集的标定文件的路径, 例如 `calib.txt`, 注意一定要用 `data_odometry_poses` 下面的 calib.txt, 否则没有 `Tr` 矩阵。
- `<gt_pose_path>` 是 KITTI 数据集的 ground truth 位姿文件的路径, 例如 `poses.txt`
- `<bag_path>` 是输出的 bag 文件的路径, 例如 `bags/kitti_00_all.bag`

输出 Topic:

- `/velodyne_points` 是 LiDAR 数据, frame_id 是 `velodyne`
- `/camera/color/left/image_raw` 是左目图像数据, frame_id 是 `camera_color_left`
- `/camera/color/left/camera_info` 是左目图像的相机内参, frame_id 是 `camera_color_left`
- `/camera/color/right/image_raw` 是右目图像数据, frame_id 是 `camera_color_right`
- `/camera/color/right/camera_info` 是右目图像的相机内参, frame_id 是 `camera_color_right`
- `/tf_static` 是静态的 TF 数据
- `/ground_truth/path` 是 ground truth 的路径数据, frame_id 是 `map`
- `/ground_truth/pose` 是 ground truth 的位姿数据, frame_id 是 `map`


## 注意

- 目前只支持 `jazzy` 和 `humble` 版本的 ros2, 其他版本没有测试过。也不在支持范围中。
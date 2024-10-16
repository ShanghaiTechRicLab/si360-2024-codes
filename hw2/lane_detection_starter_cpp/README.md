# 使用说明

这是使用 C++ 进行车道线检测的示例代码，你可以在这个基础上实现自己的车道线检测算法。

## 1. 安装依赖

```bash
sudo apt-get install libopencv-core-dev libopencv-highgui-dev libopencv-imgproc-dev
# 安装 cv_bridge
sudo apt-get install ros-[your-ros-version]-cv-bridge
```

## 2. 编译

```bash
colcon build --packages-select lane_detection_starter_cpp
```


## 3. 运行

你需要三个终端，一个播放 rosbag，一个运行 lane_detection，一个运行 rqt_image_view。

```
source install/setup.bash
ros2 bag play [your_bag_file]
```

```
source install/setup.bash
ros2 run lane_detection_starter_cpp detector
```

```
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```


## 4. 代码说明


你需要修改 `src/detector.cpp` 文件中的 `lane_detection` 函数，输出车道线信息。
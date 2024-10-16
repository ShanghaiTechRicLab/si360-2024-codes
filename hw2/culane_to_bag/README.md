# 使用说明

这是一个将culane数据集转换为ros2 bag的工具，主要用于culane数据集的训练和测试。

## 1. 安装依赖

```bash
python3 install -r requirements.txt
```

## 2. 编译

```bash
colcon build --packages-select culane_to_bag
```

## 3. 运行转换脚本

```bash
source install/setup.bash
ros2 run culane_to_bag convert /path/to/culane/dataset /path/to/output.bag
```


# ROS2 Humble 肩关节运动仿真（课程作业演示）

## 项目简介

本项目基于 **ROS2 Humble** 开发环境完成肩关节运动仿真研究。研究中，将肩关节抽象为球窝关节模型，依托几何动力学理论，完整实现了肩关节从初始位姿到目标位姿的全运动过程仿真，并实时求解了运动过程中关节的受力状态。该成果既可作为课程作业完成相关教学展示，亦是几何动力学方法在生物力学仿真领域的一次实践探索，具备良好的工程应用前景。

## 效果预览

- **起始位姿**：自然下垂
  
![起始姿态](/images/fig1.png)

- **终点位姿**：随机生成的位姿结果
  
![终点位姿](/images/fig2.png)

- **运动轨迹**：包括仿真过程的运动轨迹与受力情况
  
![动力学仿真](/images/curve.png)

- **动力学数据**：更为详细的受力数据
  
![动力学数据](/images/force.png)

## 环境依赖

### 1. 基础系统环境
|软件/系统|版本要求|
|:---:|:---:|
|操作系统|Ubuntu 22.04 LTS|
|ROS2版本|Humble|
|C++编译器|GCC 11+/Clang 14+|
|Python版本|3.10+|

### 2. 第三方依赖安装
**数学库**：Eigen3+Sophus

    sudo apt install libeigen3-dev libsophus-dev

**绘图依赖**：numpy+matplotlib+pandas

    pip3 install numpy matplotlib transforms3d

## 快速运行步骤

### 1. 工作空间搭建
    # 创建工作空间
    mkdir -p ~/ws/src
    cd ~/ws/src

    # 克隆代码到本地
    git clone https://github.com/YH563/ShoulderSimulation.git

    # 回到工作空间根目录
    cd ~/ws

### 2. 编译项目
    # 编译项目
    colcon build
    # 刷新 ROS2 环境
    source install/setup.bash

### 3. 运行实例
    # 启动Rviz2界面
    ros2 launch shoulder display.launch.py

    # 打开另外一个终端，进入工作空间根目录，启动计算与绘图节点
    ros2 launch shoulder calculator.launch.py

## 许可证
本项目采用 **MIT** 许可证，详情见 `LICENSE` 文件。
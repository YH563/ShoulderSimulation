# shoulder_msgs/msg/State.msg
# 肩关节仿真完整状态消息

# 标准消息头，包含时间戳和参考坐标系
std_msgs/Header header

# 仿真时间 (从轨迹起点开始)
float64 simulation_time

# 核心输出：大臂本体坐标系{B}相对于惯性坐标系{W}的位姿
# 这是你轨迹生成器的直接输出
geometry_msgs/Pose pose

# 核心输出：在惯性系{W}下，由“肌肉”提供的净力旋量 (已减重力)
# 这是你动力学求解器的直接输出
geometry_msgs/Wrench net_wrench
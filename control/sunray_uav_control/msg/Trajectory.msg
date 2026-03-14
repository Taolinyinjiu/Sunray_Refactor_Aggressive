# 轨迹参考类型，与参数文件 trajectory_type 保持一致
uint8 FLAT_OUTPUT=0
uint8 SE3=1

# 微分平坦轨迹使用的最高导数阶次
# 仅当 reference_type=FLAT_OUTPUT 时有效
uint8 ACCELERATION=0
uint8 JERK=1
uint8 SNAP=2

Header header

# 轨迹参考类型
uint8 reference_type

# 微分平坦轨迹的导数阶次
uint8 flat_output_order

uav_control/TrajectoryPoint[] points

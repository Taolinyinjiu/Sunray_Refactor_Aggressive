# 字段有效位掩码
uint32 FIELD_UNDEFINED=0
uint32 FIELD_POSITION=1
uint32 FIELD_ORIENTATION=2
uint32 FIELD_VELOCITY=4
uint32 FIELD_ACCELERATION=8
uint32 FIELD_JERK=16
uint32 FIELD_SNAP=32
uint32 FIELD_BODYRATES=64
uint32 FIELD_ANGULAR_ACCELERATION=128
uint32 FIELD_ANGULAR_JERK=256
uint32 FIELD_ANGULAR_SNAP=512
uint32 FIELD_HEADING=1024
uint32 FIELD_HEADING_RATE=2048
uint32 FIELD_HEADING_ACCELERATION=4096

# 相对轨迹起点时间
duration time_from_start

# 公共字段,单位为米(m)
geometry_msgs/Point position

# SE3 轨迹有效：完整姿态,单位四元数
geometry_msgs/Quaternion orientation

# 平动导数,长度单位为米(m)，时间单位为秒(s)，对时间一阶导为m/s，二阶导为m/s^2
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 snap

# SE3 轨迹有效：机体系角运动量,旋转单位为弧度(rad)，时间单位为秒(s)，对时间一阶导定义为rad/s
geometry_msgs/Vector3 bodyrates
geometry_msgs/Vector3 angular_acceleration
geometry_msgs/Vector3 angular_jerk
geometry_msgs/Vector3 angular_snap

# 微分平坦轨迹有效：世界系航向及其导数,旋转单位为弧度(rad)，时间单位为秒(s)，对时间一阶导定义为rad/s
float64 heading
float64 heading_rate
float64 heading_acceleration

# 字段有效位集合，0 表示未显式声明，由接收端按非零字段兜底推断
uint32 valid_mask

# 轨迹点ID，可用于覆盖/抢占/调试
uint32 trajectory_id

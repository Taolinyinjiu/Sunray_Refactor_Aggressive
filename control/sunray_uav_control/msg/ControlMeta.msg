# 控制源标识枚举
# 实际仲裁优先级由参数文件配置决定，而不是由消息直接声明
uint8 SOURCE_UNSPECIFIED=0
uint8 SOURCE_API=2
uint8 SOURCE_MISSION=3
uint8 SOURCE_PLANNER=4
uint8 SOURCE_RC=5
uint8 SOURCE_SAFETY=255

# 元信息时间戳
std_msgs/Header header

# 控制源ID，取值见上面的枚举
uint8 source_id

# 同一控制源内部递增的序号，用于丢弃过期命令
uint32 sequence_id

# 命令超时时间，<=0时使用该控制源的默认配置
duration timeout

# 是否允许当前命令抢占低优先级的活动控制源
bool allow_preempt

# 是否允许同一控制源的新命令覆盖旧命令
bool replace_same_source

# common

`common` 目录结构如下:

- `px4_bridge`: 外部与 PX4 飞控之间的桥梁, 用于快速读取飞控数据, 写入或查询 PX4 参数.
- `sunray_common`: 通用函数与公共工具.
- `sunray_log`: Sunray 项目中使用的日志库.
- `sunray_msgs`: Sunray 项目中使用的 ROS 话题消息定义，用于外部向Sunray_FSM状态机发送控制指令
- `sunray_srvs`: Sunray 项目中使用的 ROS 服务定义，用于外部向Sunray_FSM状态机发送控制指令

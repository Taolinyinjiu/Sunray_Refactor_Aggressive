# Sunray Helper

`Sunray_Helper` 的目标是为上层应用提供一套更友好的无人机控制接口。
当前版本中，`helper` 只建议用于已经在本文档中明确标记为“可用”的接口。

## 接口语义

- `*_async`：异步接口，请求发送成功后立即返回，不等待动作完成。
- `*_block`：阻塞接口，请求发送成功后，会继续等待状态机或 PX4 侧条件满足后再返回。
- `list_async`：多点异步接口当前通过 `trajectory` 控制链路实现。
- `list_block`：多点阻塞接口当前按顺序逐点执行，前一点完成后再发送下一点。

## 当前可用接口

### 1. 基础飞行控制

- `takeoff_async()`
- `takeoff_block()`
- `takeoff_async(double relative_takeoff_height, double max_takeoff_velocity)`
- `takeoff_block(double relative_takeoff_height, double max_takeoff_velocity)`

说明：
- `takeoff_block` 会等待状态机进入 `HOVER` 后返回。
- 当输入高度或速度小于等于 `0` 时，底层会回退到参数文件中的默认值。

- `land_async()`
- `land_block()`
- `land_async(int land_type, double land_max_velocity)`
- `land_block(int land_type, double land_max_velocity)`

说明：
- `land_block` 会等待状态机回到 `OFF`，并进一步等待 PX4 报告已落地。

- `return_async()`
- `return_block()`
- `return_async(Eigen::Vector3d target_position)`
- `return_block(Eigen::Vector3d target_position)`
- `return_async(Eigen::Vector3d target_position, double target_yaw)`
- `return_block(Eigen::Vector3d target_position, double target_yaw)`
- `return_async(Eigen::Vector3d target_position, double target_yaw, int land_type)`
- `return_block(Eigen::Vector3d target_position, double target_yaw, int land_type)`
- `return_async(Eigen::Vector3d target_position, double target_yaw, int land_type, double land_max_velocity)`
- `return_block(Eigen::Vector3d target_position, double target_yaw, int land_type, double land_max_velocity)`

说明：
- `return_block` 现在会等待完整返航流程结束，也就是等待返航后的降落完成。
- `land_type` 在返航接口中已经不再真正生效，只保留兼容参数位，当前会被忽略。

### 2. 位置控制

- `set_position_async(Eigen::Vector3d target_position)`
- `set_position_block(Eigen::Vector3d target_position)`
- `set_position_async(Eigen::Vector3d target_position, double target_yaw)`
- `set_position_block(Eigen::Vector3d target_position, double target_yaw)`
- `set_position_async(Eigen::Vector3d target_position, double target_yaw, double target_yaw_rate)`
- `set_position_block(Eigen::Vector3d target_position, double target_yaw, double target_yaw_rate)`

说明：
- `set_position_block` 当前会等待状态机从 `POSITION_CONTROL` 自动切回 `HOVER`。
- 也就是说，阻塞完成语义以 FSM 判定为准，而不是 helper 本地再做一套独立的“到点”判定。
- 当前 position service 底层只直接支持位置和 yaw，不直接支持严格的 `yaw_rate` 约束。
- 因此带 `target_yaw_rate` 的单点阻塞/异步接口，目前只保证位置与 yaw，`yaw_rate` 会被忽略并打印警告。

### 3. 多点位置控制

- `set_position_list_async(std::vector<Eigen::Vector3d> target_position_list)`
- `set_position_list_block(std::vector<Eigen::Vector3d> target_position_list)`
- `set_position_list_async(std::vector<std::pair<Eigen::Vector3d, double>> target_position_list)`
- `set_position_list_block(std::vector<std::pair<Eigen::Vector3d, double>> target_position_list)`
- `set_position_list_async(std::vector<std::pair<Eigen::Vector3d, double>> target_position_list, double target_yaw_rate)`
- `set_position_list_block(std::vector<std::pair<Eigen::Vector3d, double>> target_position_list, double target_yaw_rate)`

说明：
- `list_async` 当前会转换成 `uav_control/Trajectory` 后发送到 `trajectory` 控制链路。
- `list_block` 当前不是一条轨迹内部阻塞，而是逐点调用单点阻塞接口。
- 带 `target_yaw_rate` 的 `list_async` 会把 `heading_rate` 一起写入轨迹点。
- 带 `target_yaw_rate` 的 `list_block` 仍然只保证位置与 yaw，不保证严格的 `yaw_rate`。

### 4. 速度与位置-速度控制

- `set_linear_velocity_async(Eigen::Vector3d target_velocity)`
- `set_position_velocity_async(Eigen::Vector3d target_position, double target_velocity)`
- `set_position_velocity_block(Eigen::Vector3d target_position, double target_velocity)`
- `set_position_velocity_list_async(std::vector<std::pair<Eigen::Vector3d, double>> target_position_list)`
- `set_position_velocity_list_block(std::vector<std::pair<Eigen::Vector3d, double>> target_position_list)`

说明：
- `set_linear_velocity_async` 当前为高频速度控制接口，不带阻塞版本。
- `set_position_velocity_async` 会向速度控制链路发送“位置目标 + 期望速度”。
- `set_position_velocity_block` 会在发送成功后等待位置到达。
- `set_position_velocity_list_async` 当前会转换成 `uav_control/Trajectory`。
- `set_position_velocity_list_block` 当前按顺序逐点执行。

### 5. Yaw 控制

- `set_yaw_async(double target_yaw)`
- `set_yaw_block(double target_yaw)`
- `set_yaw_adjust_async(double adjust_yaw)`
- `set_yaw_adjust_block(double adjust_yaw)`

说明：
- `set_yaw_block` 和 `set_yaw_adjust_block` 会等待 yaw 到达。
- yaw 到达阈值由参数 `yaw_reached_tolerance_rad` 控制。

### 6. 轨迹控制

- `set_trajectory_point_async(const uav_control::TrajectoryPoint &target_trajpoint)`
- `set_trajectory_async(const uav_control::Trajectory &trajectory)`

说明：
- 这是当前推荐的多字段高频参考输入接口。
- 多点轨迹会进入状态机的 `TRAJECTORY_CONTROL`，并由控制器内部轨迹缓存进行采样。

### 7. 查询接口

- `get_uav_odometry()`
- `get_uav_position()`
- `get_uav_velocity_linear()`
- `get_uav_velocity_angular()`
- `get_uav_attitude_rpy_rad()`
- `get_uav_attitude_rpy_deg()`
- `get_uav_attitude_quat()`
- `get_target_position()`
- `get_target_velocity_linear()`
- `get_target_velocity_angular()`
- `get_target_attitude_rpy_rad()`
- `get_target_attitude_rpy_deg()`
- `get_target_attitude_quat()`
- `get_statemachine_state()`

补充说明：
- `get_target_thrust()` 当前没有真实外部 thrust 控制链路支撑，因此默认返回 `0.0`，不建议依赖。

## 当前不可用接口

以下接口已经在代码中明确标记为“当前不可用”，调用时会打印警告并返回 `false`：

- `set_velocity_area(Eigen::Vector3d protect_area)`
- `clear_velocity_area()`
- `set_anglar_velocity_async(Eigen::Vector3d target_velocity)`
- `set_attitude_thrust_async(Eigen::Vector3d attitude, double thrust)`
- `set_bodyrate_thrust_async(Eigen::Vector3d bodyrate, double thrust)`
- `is_external_attitude_thrust_ready()`
- `set_complex_control()`

原因概括：
- 电子围栏保护区逻辑尚未接入控制器/FSM。
- 外部姿态/推力接管、预热、超时回切链路尚未实现。
- `COMPLEX_CONTROL` 在 FSM 中已接收消息，但尚未连到实际控制输出通路。

## 当前建议

- 低频任务控制优先使用：`takeoff_*`、`land_*`、`return_*`、`set_position_*`
- 多点任务优先使用：`set_trajectory_async`器
- 高层业务如果需要“执行完成”语义，优先使用 `*_block`
- 如果需要真正的高频连续控制，优先使用 `trajectory` 或 `velocity` 通路，而不是反复调用阻塞接口

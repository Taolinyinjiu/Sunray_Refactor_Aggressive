/**
 * @file mavros_helper.hpp
 * @brief 将mavros的接口进行抽象，向控制器提供快速的控制方式
 * @details
    1. 订阅绝大部分Mavros的状态话题（参考原来的Sunray项目框架）
    2. 发布大部分Mavros的控制话题/服务
    3. Vision_pose话题
    4. 发布PX4State话题，属于自定义消息sunray_msgs::PX4_State
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-25
 * @version 0.1
 *
 *
 */

#include "control_data_types/mavros_helper_data_types.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include "geometry_msgs/Pose.h"
#include "mavros_msgs/BatteryStatus.h"

// 首先，设计这个类，目的是为了将mavros进行抽象，将mavros的接口进行封装，提供给控制器使用
// 这里存在着三层架构
// Sunray_FSM(控制模块状态机，与ROS相关，但是与mavros解耦，处理各类回调函数与逻辑)
// Sunray_Controller(控制器，与ros解耦，但是与mavros耦合，处理具体的控制指令),其中的具体算法与ros解耦，设计为单独的类函数
// Mavros_Helper(辅助类，与mavros耦合，提供给控制器使用的px4控制接口)

// 其次，Mavros_Helper并不单独成为一个节点，而是作为一个类，进入到Sunray_Controller中作为一个成员变量使用
// Sunray_Controller也不单独称为一个节点，而是作为一个类，进入到Sunray_FSM中作为一个成员变量使用

// 总的来说，整个sunray_uav_control模块，只存在一个节点，就是sunray_fsm_node，当然也可以叫做sunray_uav_control_node
// 既然Sunray_FSM对mavros进行了解耦，那么PX4_State这个话题，就应该在Mavros_Helper和Sunray_Controller中进行发布
// PX4_State主要是对px4状态的收集然后发布，提供给Sunray框架中的所有模块使用，谁用谁订阅
// 那么这里就涉及到一个数据新鲜度的问题，我们如何保证每次更新的数据都是新的？作为一个单独的类，回调的频率必然受到sunray_fsm自身频率的限制
// 多线程之间必然涉及到数据同步的问题，我们需要引入线程锁来防止数据竞争
// 但这并不是我们想要的，我们提到Sunray_Helper自身回调频率500Hz主要是为了，获取到的数据都是新的，也就是说不受缓冲队列的干扰
// 所以我们直接设置缓冲队列为1就好
//

// 第二个问题是，我们如何返回获取的数据，或者说控制器如何高效的利用我们得到的数据？

// 这个结构体，用途是配置MavrosHelper的初始化过程,根据3.25的讨论，我们认为这个MavrosHelper可能会被别的模块所使用，
// 但未必是对所有的px4话题数据感兴趣，因此我们提供一个ConfigList用于配置订阅那些Mavros数据
// 1. state 基本信息和扩展信息(对应 mavros/state + mavros/externed_state)
// 2. battery 电池信息(对应 mavros/battery ，但是在simulator中没有数据被发布)
// 3. ekf2状态(对应 mavros/estimator_status)
// 4. 无人机当前里程计状态(对应 mavros/local_position/odom)
// 5. 无人机运动控制权限(这里指的是是否初始化出控制句柄，比如说mavros的服务端和setpoint的发布者)
// 6. 无人机运动目标信息(对应setpoint_raw/target_local 和 setpoint_raw/target_attitude)
// 7. 无人机姿态 (对应 mavros/imu/data)
//
struct MavrosHelper_ConfigList {
    bool state = true;
    bool battery = true;
    bool ekf2_status = false;
    bool local_odom = true;
    bool local_attitude = false;
    bool uav_control_handle = false;
    bool uav_target_state = false;
    MavrosHelper_ConfigList() = default;
    MavrosHelper_ConfigList(bool all_change);
};

inline MavrosHelper_ConfigList::MavrosHelper_ConfigList(bool all_change) {
    state = all_change;
    battery = all_change;
    ekf2_status = all_change;
    local_odom = all_change;
    local_attitude = all_change;
    uav_control_handle = all_change;
    uav_target_state = all_change;
}

class MavrosHelper {
    explicit MavrosHelper(MavrosHelper_ConfigList config_list = MavrosHelper_ConfigList());
    ~MavrosHelper() = default;

  public:
    // 返回飞控基本状态
    control_common::Mavros_State get_state();
    // 返回飞控电池状态
    control_common::Mavros_Battery get_battery();
    // 返回飞控ekf2估计器的状态
    control_common::Mavros_Estimator get_estimator_status();
    // 返回飞控里程计状态
    control_common::UAVStateEstimate get_odometry();
    // 返回飞控当前姿态
    Eigen::Vector3d get_attitude_eluer_rad();
    Eigen::Vector3d get_attitude_eluer_deg();
    Eigen::Quaterniond get_attitude_quat();
    // 获取无人机偏航角
    float get_yaw_rad();
    float get_yaw_deg();
    // 返回飞控当前运动目标
    control_common::Mavros_SetpointLocal get_target_local();
    control_common::Mavros_SetpointAttitude get_target_attitude();
    /*----------------以下部分与uav_control强相关----------------- */
    // 发布vision_pose 向mavros进行融合,返回发布成功或者失败
    bool pub_vision_pose(control_common::UAVStateEstimate uav_state);
    // 切换px4的模式，返回切换成功或者失败
    bool set_px4_mode(control_common::FlightMode px4_mode);
    // 解锁飞控,true解锁，false上锁
    bool set_arm(bool arm_state);
    // 向mavros发布setpoint数据
    bool pub_local_setpoint(control_common::Mavros_SetpointLocal setpoint_local);
    bool pub_attitude_setpoint(control_common::Mavros_SetpointAttitude setpoint_attitude);

  private:
    // ROS话题订阅者
    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber estimator_sub_;
    ros::Subscriber local_odom_sub_;
    ros::Subscriber local_attitude_sub_;
    ros::Subscriber setpoint_local_sub_;
    ros::Subscriber setpoint_attitude_sub_;
    // TODO: 这里应该还有与GPS相关的
};

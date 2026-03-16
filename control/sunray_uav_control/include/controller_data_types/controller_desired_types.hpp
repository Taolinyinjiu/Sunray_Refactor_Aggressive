/* clang-format off */
/**
 * @file Desired_State.hpp
 * @author Taolinyinjiu@Yun-Drone Tech
 * @brief 本文件旨在描述Sunray项目中控制器模块所需要遵循的数据类型，Desired_State表示控制器在进行更新状态量时的期望变量
 *	1. 在仔细的分析后，我们认为sunray_attitude_controller承袭了px4_ctrl_controller中的部分数据结构，因此这里对px4_ctrl中的controller做一个简单地分析，该控制器实现了一个级联的PID结构
 *		1.1 外环：根据位置和速度的误差，计算所需要的加速度
 *		1.2 内环：将期望的加速度转换为姿态角和推力
 *	2. 高阶轨迹可以作为统一的上层期望输入，但不能作为所有控制器唯一的输入语义，对于姿态控制器来说，原因有这样几个
 *		2.1 对于轨迹跟踪控制器，高阶轨迹很合适，比如位置控制、速度-位置级联、微分平坦控制、几何控制，这些控制器天然就吃 pos/vel/acc，有些还会用 jerk，所以用高阶轨迹做统一输入很舒服
 *		2.2 对于内环或者非轨迹控制器，比如姿态控制器，角速度控制器，推力分配控制器，他们的输入量更像是q/bodyrate/thrust，高阶轨迹只能作为他们的参考，而不是本质的输入
 *		2.3 综上，我们认为高阶轨迹作为参考层次，每个控制器从参考层映射到自己的输入中去，也就是从高阶轨迹总取出自己所需要的那部分
 *	3. 由于后续我们并不准备实现SE3 Control以及对应的求解器，因此选择使用微分平坦轨迹作为后续所有控制器的参考输入，也就是期望状态，并修改命名，将Desired_State_t修改为FlatTrajectoryPoint 
 *  4. 使用命名空间包裹，和sunray_uav_control现有语义一致
 *
 * @version 0.1
 * @date 2026-03-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* clang-format on */
#pragma

#include <Eigen/Dense>

namespace controller_data_types {

// FlatTrajectoryPoint 各字段的有效位定义，
enum FlatTrajectoryField : uint32_t {
  kPositionValid = 1u << 0,     // position 字段有效
  kVelocityValid = 1u << 1,     // velocity 字段有效
  kAccelerationValid = 1u << 2, // acceleration 字段有效
  kJerkValid = 1u << 3,         // jerk 字段有效
  kYawValid = 1u << 4,          // yaw 字段有效
  kYawRateValid = 1u << 5,      // yaw_rate 字段有效
};
// 单个时刻的平坦轨迹参考点，供高层轨迹跟踪控制器使用。
struct FlatTrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position = Eigen::Vector3d::Zero(); // 期望位置，世界坐标系
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // 期望速度，世界坐标系
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(); // 期望加速度
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();         // 期望加加速度

  double yaw = 0.0;      // 期望偏航角
  double yaw_rate = 0.0; // 期望偏航角速度

  // 用于区分“字段未提供”和“字段有效但数值恰好为 0”
  uint32_t valid_fields = kPositionValid | kYawValid;
};

} // namespace controller_data_types
/**
 * @file position_point.hpp
 * @brief 设计一个结构体，用于描述helper在传递基础位置移动命令时，需要的函数入口参数
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 * 
 */

#pragma once

#include <Eigen/Dense>
#include <optional>

struct MovePoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    std::optional<double> velocity;
    std::optional<double> yaw;
    std::optional<double> yaw_rate;
};

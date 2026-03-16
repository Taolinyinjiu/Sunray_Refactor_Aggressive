/**
 * @file Desired_State.hpp
 * @author Taolinyinjiu@Yun-Drone Tech
 * @brief 本文件旨在描述Sunray项目中控制器模块所需要遵循的数据类型，Desired_State表示控制器在进行更新状态量时的期望变量
 * @version 0.1
 * @date 2026-03-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma 

#include <Eigen/Dense>

struct Desired_State_t{
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d jerk;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;
}


#pragma once

#include <string>

struct SunrayHelperConfig {
  double takeoff_wait_s = 30 ;
  double land_wait_s = 30 ;
  int takeoff_timeout_operate = 2;
  int land_timeout_operate = 0;
};

// 读取默认的sunray_helper_config.yaml文件的路径
std::string getDefaultConfigPath();
// 加载yaml文件的参数，并返回一个SunrayHelperConfig结构体，用于确定在状态超时的时候Helper作出什么应对方案
SunrayHelperConfig
loadConfig(const std::string &yaml_path = getDefaultConfigPath());


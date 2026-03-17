#include <gtest/gtest.h>

#include "sunray_helper/sunray_helper.h"

TEST(SunrayHelperCompileTest, UavInfoCanConvertToRosMessage) {
  SunrayUavInfo info;
  EXPECT_FALSE(info.toRosMessage().data.empty());
}

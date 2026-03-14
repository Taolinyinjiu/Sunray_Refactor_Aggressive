#include "sunray_helper/sunray_helper.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_helper_hover_land_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Sunray_Helper helper(nh);
  const char *tag = "HelperHoverLandDemo";

  if (!helper.takeoff_async() ||
      !sunray_demo::wait_for_state(helper, sunray_fsm::SunrayState::HOVER,
                                   15.0)) {
    return sunray_demo::fail(tag, "takeoff_async");
  }

  ros::Duration(5.0).sleep();

  if (!helper.land_async() ||
      !sunray_demo::wait_for_state(helper, sunray_fsm::SunrayState::OFF,
                                   25.0)) {
    return sunray_demo::fail(tag, "land_async");
  }

  return sunray_demo::done(tag, "done");
}

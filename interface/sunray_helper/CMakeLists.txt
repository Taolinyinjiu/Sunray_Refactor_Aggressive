cmake_minimum_required(VERSION 3.0.2)
project(sunray_helper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  sunray_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sunray_helper
  CATKIN_DEPENDS nav_msgs roscpp std_msgs sunray_common
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(sunray_helper
  src/sunray_helper.cpp
  src/state_reader.cpp
  src/command_sender.cpp
)
target_link_libraries(sunray_helper
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(sunray_helper_compile_test
    test/sunray_helper_compile_test.cpp
  )
  if(TARGET sunray_helper_compile_test)
    target_link_libraries(sunray_helper_compile_test
      sunray_helper
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()
endif()

add_executable(minimal_takeoff_example
  examples/minimal_takeoff.cpp
)
target_link_libraries(minimal_takeoff_example
  sunray_helper
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(goto_waypoint_example
  examples/goto_waypoint.cpp
)
target_link_libraries(goto_waypoint_example
  sunray_helper
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

install(TARGETS sunray_helper minimal_takeoff_example goto_waypoint_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

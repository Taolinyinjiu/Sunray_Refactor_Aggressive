cmake_minimum_required(VERSION 3.0.2)
project(sunray_uav_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(controller_output_types_compile_test
    test/controller_output_types_compile_test.cpp
  )
  if(TARGET controller_output_types_compile_test)
    target_link_libraries(controller_output_types_compile_test
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()
endif()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

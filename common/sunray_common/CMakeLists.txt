cmake_minimum_required(VERSION 3.0.2)
project(sunray_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometry_eigen_conversions quad_state_estimate
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(geometry_eigen_conversions
  src/geometry_eigen_conversions.cpp
)
target_link_libraries(geometry_eigen_conversions
  ${catkin_LIBRARIES}
)

add_library(quad_state_estimate
  src/quad_state_estimate.cpp
)
target_link_libraries(quad_state_estimate
  geometry_eigen_conversions
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(sunray_common_ros_compile_test_node
    test/sunray_common_ros_compile_test_node.cpp
  )
  target_link_libraries(sunray_common_ros_compile_test_node
    quad_state_estimate
    geometry_eigen_conversions
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )

  catkin_add_gtest(geometry_eigen_conversions_ut
    test/test_geometry_eigen_conversions.cpp
  )
  if(TARGET geometry_eigen_conversions_ut)
    target_link_libraries(geometry_eigen_conversions_ut
      geometry_eigen_conversions
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()

  catkin_add_gtest(quad_state_estimate_ut
    test/test_quad_state_estimate.cpp
  )
  if(TARGET quad_state_estimate_ut)
    target_link_libraries(quad_state_estimate_ut
      quad_state_estimate
      geometry_eigen_conversions
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()
endif()

install(TARGETS geometry_eigen_conversions quad_state_estimate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

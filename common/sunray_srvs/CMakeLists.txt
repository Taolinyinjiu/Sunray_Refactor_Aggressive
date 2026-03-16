cmake_minimum_required(VERSION 3.0.2)
project(sunray_srvs)

add_compile_options(-std=c++14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "srv/*.srv")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

if(SRV_FILES)
  add_service_files(
    FILES
    ${SRV_FILES}
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    message_runtime
  )
else()
  catkin_package()
endif()

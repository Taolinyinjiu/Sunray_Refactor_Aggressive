cmake_minimum_required(VERSION 3.0.2)
project(sunray_msgs)

add_compile_options(-std=c++14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "msg/*.msg")

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

if(MSG_FILES)
  add_message_files(
    FILES
    ${MSG_FILES}
  )

  generate_messages()

  catkin_package(
    CATKIN_DEPENDS message_runtime
  )
else()
  catkin_package()
endif()

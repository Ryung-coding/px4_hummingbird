cmake_minimum_required(VERSION 3.8)
project(px4_hummingbird_cmd)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

find_package(dynamixel_sdk REQUIRED)

add_executable(px4_position_cmd
  src/px4_position_cmd.cpp
)

add_executable(px4_servo_to_gz
  src/px4_servo_to_gz.cpp
)

add_executable(px4_servo_to_dynamixel
  src/px4_servo_to_dynamixel.cpp
)

target_include_directories(px4_position_cmd PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(px4_servo_to_gz PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(px4_servo_to_dynamixel PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(px4_position_cmd
  Eigen3::Eigen
)

target_link_libraries(px4_servo_to_gz
  Eigen3::Eigen
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

target_link_libraries(px4_servo_to_dynamixel
  Eigen3::Eigen
  dynamixel_sdk::dynamixel_sdk
)

ament_target_dependencies(px4_position_cmd
  rclcpp
  px4_msgs
)

ament_target_dependencies(px4_servo_to_gz
  rclcpp
  px4_msgs
)

ament_target_dependencies(px4_servo_to_dynamixel
  rclcpp
  px4_msgs
)

install(TARGETS
  px4_position_cmd
  px4_servo_to_gz
  px4_servo_to_dynamixel
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/px4_multirotor_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
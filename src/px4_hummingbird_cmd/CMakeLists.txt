cmake_minimum_required(VERSION 3.8)
project(px4_hummingbird_cmd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(px4_position_cmd src/px4_position_cmd.cpp)
add_executable(px4_servo_to_gz src/px4_servo_to_gz.cpp)

target_include_directories(px4_position_cmd PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(px4_position_cmd
  Eigen3::Eigen
)

ament_target_dependencies(px4_position_cmd
  rclcpp
  px4_msgs
)

target_link_libraries(px4_servo_to_gz
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

ament_target_dependencies(px4_servo_to_gz
  rclcpp
  px4_msgs
)

install(TARGETS
  px4_position_cmd
  px4_servo_to_gz
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

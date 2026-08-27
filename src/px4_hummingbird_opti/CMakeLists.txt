cmake_minimum_required(VERSION 3.8)
project(px4_hummingbird_opti)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mocap_to_px4
  src/mocap_to_px4.cpp
)

target_include_directories(mocap_to_px4 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mocap_to_px4
  Eigen3::Eigen
)

ament_target_dependencies(mocap_to_px4
  rclcpp
  geometry_msgs
  px4_msgs
)

install(TARGETS
  mocap_to_px4
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  include
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(px4_hummingbird_avp)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/avp_bridge.py
  scripts/avp_trajectory_logger.py
  scripts/lics_avp_planner.py
  scripts/lics_corridor_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

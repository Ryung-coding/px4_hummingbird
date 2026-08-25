cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ActuatorMotors.msg
  msg/ActuatorServos.msg
  msg/VehicleAttitude.msg
  msg/VehicleAttitudeSetpoint.msg
  msg/HummingbirdStatus.msg
  msg/VehicleLocalPosition.msg
  msg/VehicleLocalPositionSetpoint.msg
  msg/VehicleOdometry.msg
  msg/ManualControlSetpoint.msg
  msg/OffboardControlMode.msg
  msg/TrajectorySetpoint.msg
  msg/TrajectorySetpoint6dof.msg
  msg/VehicleCommand.msg
  msg/VehicleStatus.msg
  msg/VehicleThrustSetpoint.msg
  msg/VehicleTorqueSetpoint.msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

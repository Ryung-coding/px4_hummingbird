# HummingBird external servo kill flag

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

bool kill # true requests external servo torque off through the ROS/DDS bridge

# TOPICS servo_kill

# HummingBird command/control mode status

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

uint8 hb_cmd_source
uint8 HB_CMD_SOURCE_RC = 0
uint8 HB_CMD_SOURCE_DDS = 1

uint8 hb_ctrl_mode
uint8 HB_CTRL_MODE_CONVENTIONAL = 0
uint8 HB_CTRL_MODE_FULLY_ACTUATED = 1

bool hummingbird_airframe
bool dds_command_enabled
bool fully_actuated_control_enabled

# TOPICS hummingbird_status

############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE drivers__actuators__dynamixel
	MAIN dynamixel
	SRCS
		DynamixelDriver.cpp
		DynamixelDriver.hpp
		DynamixelProtocol.cpp
		DynamixelProtocol.hpp
	MODULE_CONFIG
		module.yaml
	)

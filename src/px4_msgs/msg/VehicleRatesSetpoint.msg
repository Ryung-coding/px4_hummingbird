uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# Body angular rates in FRD frame
float32 roll  # [rad/s] roll rate setpoint
float32 pitch # [rad/s] pitch rate setpoint
float32 yaw   # [rad/s] yaw rate setpoint

# Normalized thrust command in body NED frame [-1,1]
float32[3] thrust_body

bool reset_integral # Reset roll/pitch/yaw integrals (navigation logic change)

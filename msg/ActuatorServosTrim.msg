# Servo trims, added as offset to servo outputs
uint64 timestamp			# time since system start (microseconds)

uint8 NUM_CONTROLS = 6
float32[6] trim    # range: [-1, 1]

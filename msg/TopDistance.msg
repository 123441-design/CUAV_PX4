# Four measured upward distances from the active vehicle sensor interface.
# Sensor order: front-right, front-left, rear-right, rear-left.

uint64 timestamp
uint64[4] timestamp_sample
float32[4] distance_m
uint16 sequence
uint8 valid_mask

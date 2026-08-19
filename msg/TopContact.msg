# Debounced input source is handled by custom_action_control. The future STM32
# UART driver only needs to publish this raw, timestamped contact state.

uint64 timestamp
bool contact
bool valid


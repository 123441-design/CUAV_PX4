# Top Distance Bridge

`top_distance_bridge` is the dedicated physical receiver for the four upward
laser measurements. On CUAV V6X it owns TELEM1 (`/dev/ttyS6`) at 115200 baud.
It is independent of `mylink_bridge`, which remains on TELEM2 for WiFi and
upper-computer control traffic.

The sensor MCU sends one MAVLink 2 `PING` targeted to system `1`, component
`25` for each four-sensor measurement cycle. `PING.time_usec` contains four
little-endian `uint16_t` millimetre values in this order: front-right,
front-left, rear-right, rear-left. `PING.seq` contains:

```text
bits 31..28  marker 0xA
bits 27..24  version 1
bits 23..20  four-sensor validity mask
bits 19..16  reserved, must be zero
bits 15..0   measurement-frame sequence
```

Valid packets are converted to metres and published atomically as one
`top_distance` uORB report. The receiver runs while disarmed as well as armed;
flight-state validation remains in `custom_action_control`.

The module does not accept flight-control commands and does not transmit any
MAVLink response on TELEM1.

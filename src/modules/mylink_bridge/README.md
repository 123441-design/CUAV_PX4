# MyLink MAVLink bridge

`mylink_bridge` owns the serial port selected by `MLB_CONFIG` and parses a
binary MAVLink 1/2 byte stream with the generated PX4 MAVLink C library. It
does not accept newline-delimited text commands.

On CUAV V6X this firmware assigns TELEM2 (`/dev/ttyS4`) to the bridge at
115200 baud. The independent `top_distance_bridge` owns TELEM1 for the physical
four-laser stream. USB MAVLink
remains available for QGC.

## Four-distance monitoring output

The bridge also subscribes to the unified `top_distance` uORB topic. Once an
upper computer is present, it sends the newest four-sensor frame over the same
MyLink transport at no more than 10 Hz. Real hardware therefore follows
TELEM1/`top_distance_bridge` -> uORB -> TELEM2/WiFi, while SITL follows Gazebo
four sensors -> `gz_bridge` -> uORB -> UDP port 14541. No separate SITL relay
script is required.

All four millimetre values remain in one MAVLink 2 `PING` frame using the same
marker/version/validity/sequence layout accepted by `top_distance_bridge`.
Sensor order is front-right, front-left, rear-right, rear-left; the V3.1 GUI
reorders these for display as left-up, right-up, left-down, right-down.

## Four-motor monitoring output

The bridge also samples `actuator_motors.control[0..3]` and sends Motor1 through
Motor4 in one MAVLink 2 `PING` frame at no more than 10 Hz. Each value is
clamped to `0.000..1.000` and encoded as an unsigned 16-bit integer where
`1000` means `1.000`. The metadata uses marker `0xB`, protocol version 1, four
validity bits, an armed flag and a 16-bit sequence number. When PX4 is disarmed,
all four channels are explicitly reported as `0.000`; missing or stale armed
samples are marked invalid.

The V3.1 GUI displays the four values and percentage bars. If no new frame is
received for 0.5 seconds, it shows dashes and grey bars. These values are PX4's
normalized commands to the motors, not measured RPM. Measured speed would
require ESC telemetry supported by the installed ESCs.

## Receive gate and event flags

The serial port is opened and MAVLink framing and CRC are checked from boot.

- Disarmed: valid frames are discarded without a reply. Cached data, event
  flags and the previous command are cleared when the vehicle disarms.
- Armed but not Offboard: the newest supported `COMMAND_LONG` replaces the
  previous cached command. `SET_POSITION_TARGET_LOCAL_NED` messages publish
  the Offboard availability heartbeat but no trajectory setpoint. An Offboard
  mode request is forwarded to Commander after the stream has prewarmed.
- Armed and Offboard: a cached frame is processed once on entry and new frames
  are processed immediately. Each fresh local-NED setpoint publishes both
  `offboard_control_mode` and `trajectory_setpoint`. `COMMAND_LONG` event
  messages set event bits and return a binary `COMMAND_ACK`.

The sender must stream `SET_POSITION_TARGET_LOCAL_NED` above 2 Hz (20 Hz is
recommended), starting at least one second before requesting Offboard. The
bridge intentionally does not replay a stale heartbeat, so PX4 exits Offboard
normally when the serial stream stops.

Takeoff, land, speed, pause, continue, RTL and mission-start events are only
recorded as flags. `MAV_CMD_DO_MOTOR_TEST` is the one executable event: while
ACTIVE it selects PX4 Offboard direct-actuator mode and publishes equal
normalized thrust for Motor1 through Motor4. PWM/DShot mapping remains in the
existing mixer and output driver.

Event bits:

| Bit | Mask | Event |
| ---: | ---: | --- |
| 0 | `0x00000001` | Takeoff |
| 1 | `0x00000002` | Land |
| 2 | `0x00000004` | Speed |
| 3 | `0x00000008` | Pause |
| 4 | `0x00000010` | Continue |
| 5 | `0x00000020` | RTL |
| 6 | `0x00000040` | Mission start |
| 7 | `0x00000080` | Motor throttle command accepted |

## Supported incoming MAVLink messages

| Message | Behaviour after the gate opens |
| --- | --- |
| `PING` (ID 4) | Standard target `0/0` is echoed. Targeted responses are not echoed again. |
| `COMMAND_LONG` (ID 76) | Decodes an allowed command and sets its event bit. |
| `COMMAND_INT` (ID 75) | Returns `COMMAND_ACK/MAV_RESULT_COMMAND_LONG_ONLY`. |
| `SET_POSITION_TARGET_LOCAL_NED` (ID 84) | Prewarms Offboard; publishes velocity/position setpoint only while Active. |

Allowed `COMMAND_LONG.command` values:

| MAV_CMD | Value | PX4 action |
| --- | ---: | --- |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 20 | Enter RTL. |
| `MAV_CMD_NAV_LAND` | 21 | Enter Land. |
| `MAV_CMD_NAV_TAKEOFF` | 22 | Request PX4 takeoff handling. |
| `MAV_CMD_DO_CHANGE_SPEED` | 178 | Change mission speed/throttle setpoint according to PX4 vehicle support. |
| `MAV_CMD_DO_PAUSE_CONTINUE` | 193 | Pause (`param1=0`) or continue (`param1=1`) the mission. |
| `MAV_CMD_MISSION_START` | 300 | Start the uploaded mission. |
| `MAV_CMD_DO_MOTOR_TEST` | 209 | Set Motor1 through Motor4 equally to a normalized throttle for a bounded interval. |

All command results are returned as binary MAVLink `COMMAND_ACK` (ID 77).
Unsupported commands receive `MAV_RESULT_UNSUPPORTED`. Only one command may be
pending at once.

`MAV_CMD_DO_CHANGE_SPEED.param3` is the standard MAVLink throttle percentage,
but it is not a direct four-motor output command. PX4 only applies it where the
active vehicle/mode supports a cruising-throttle setpoint.

## Motor throttle command

`MAV_CMD_DO_MOTOR_TEST` is accepted only while the bridge gate is ACTIVE
(armed and Offboard). Unlike the other event commands, it is never cached
before ACTIVE, so an old throttle value cannot execute after a later mode
transition. Its fields are:

- `param1`: first motor instance, must be `1`.
- `param2`: throttle type, must be `MOTOR_TEST_THROTTLE_PERCENT` (`0`).
- `param3`: normalized command-layer throttle, `0..100` percent.
- `param4`: mandatory timeout, greater than `0` and no more than `3` seconds.
- `param5`: motor count, must be `4`; all four motors receive the same value simultaneously.

The bridge converts `param3` to `0.0..1.0` and, for the command lifetime,
continuously publishes `offboard_control_mode.direct_actuator=true`. It waits
until Commander reports that control allocation is disabled, then continuously
publishes `actuator_motors` with Motor1..Motor4 set equally and all remaining
channels set to `NAN`. This prevents ControlAllocator and the direct command
from competing as publishers of the same motor topic during the transition.

Leaving ACTIVE Offboard or disarming publishes an explicit zero command and
immediately releases direct control. Reaching the deadline holds an explicit
zero command for 100 ms before releasing direct control, so the output driver
cannot retain the last finite throttle sample during the handover.
`COMMAND_ACK/MAV_RESULT_ACCEPTED` only means that the command was validated and
scheduled; physical output must still be verified through `actuator_outputs`
or ESC telemetry.

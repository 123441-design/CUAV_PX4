/****************************************************************************
 * Project-local protocol shared by MyLink, custom_action_control and V3.
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace custom_action_protocol
{
constexpr uint16_t kMavCmdUser1 = 31010;
constexpr uint8_t kComponentId = 25; // MAV_COMP_ID_USER1

// Current top-distance wire format: one targeted PING carries all four
// millimetre measurements in PING.time_usec (one uint16_t per sensor).
// PING.seq carries an unambiguous marker, version, validity mask and frame
// sequence. The marker/version pair prevents unrelated targeted PING messages
// from being interpreted as sensor data.
constexpr uint32_t kTopDistanceArrayMarkerMask = 0xF0000000u;
constexpr uint32_t kTopDistanceArrayMarker = 0xA0000000u;
constexpr uint32_t kTopDistanceArrayVersionMask = 0x0F000000u;
constexpr uint32_t kTopDistanceArrayVersion = 0x01000000u;
constexpr uint32_t kTopDistanceArrayValidMask = 0x00F00000u;
constexpr uint8_t kTopDistanceArrayValidShift = 20;
constexpr uint32_t kTopDistanceArrayReservedMask = 0x000F0000u;
constexpr uint32_t kTopDistanceArraySequenceMask = 0x0000FFFFu;
constexpr uint8_t kTopDistanceArrayDistanceBits = 16;
constexpr uint64_t kTopDistanceArrayMillimetresMask = 0xFFFFu;

constexpr uint8_t kTopDistanceSensorCount = 4;

// Normalized motor-monitor format: one targeted PING carries Motor1..Motor4
// as four uint16_t values in PING.time_usec. A value of 1000 represents a
// normalized actuator command of 1.000. PING.seq uses a marker distinct from
// top-distance reports and includes validity, armed state and a frame counter.
constexpr uint32_t kMotorOutputArrayMarkerMask = 0xF0000000u;
constexpr uint32_t kMotorOutputArrayMarker = 0xB0000000u;
constexpr uint32_t kMotorOutputArrayVersionMask = 0x0F000000u;
constexpr uint32_t kMotorOutputArrayVersion = 0x01000000u;
constexpr uint32_t kMotorOutputArrayValidMask = 0x00F00000u;
constexpr uint8_t kMotorOutputArrayValidShift = 20;
constexpr uint32_t kMotorOutputArrayArmedFlag = 0x00010000u;
constexpr uint32_t kMotorOutputArrayReservedMask = 0x000E0000u;
constexpr uint32_t kMotorOutputArraySequenceMask = 0x0000FFFFu;
constexpr uint8_t kMotorOutputArrayValueBits = 16;
constexpr uint16_t kMotorOutputArrayScale = 1000;
constexpr uint8_t kMotorOutputCount = 4;

enum class Command : uint8_t {
	SearchTop = 1,
	DirectionIntent = 2,
	RebaseComplete = 3,
};

enum class Direction : uint8_t {
	Forward = 1,
	Back = 2,
	Left = 3,
	Right = 4,
	Up = 5,
	Down = 6,
};

enum class Result : uint8_t {
	None = 0,
	CustomStarted = 1,
	ButtonConsumed = 2,
	LegacyAllowed = 3,
	RebaseAccepted = 4,
	HandoverPending = 5,
	ContactPressEntered = 6,
};

constexpr int32_t encodeResult(uint16_t request_id, Result result)
{
	return (static_cast<int32_t>(request_id) << 8) | static_cast<uint8_t>(result);
}
}

/****************************************************************************
 * Project-local protocol shared by MyLink, custom_action_control and V3.
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace custom_action_protocol
{
constexpr uint16_t kMavCmdUser1 = 31010;
constexpr uint8_t kComponentId = 25; // MAV_COMP_ID_USER1

// Fixed identity used by the V3.1 upper computer on the dedicated WiFi link.
constexpr uint8_t kUpperComputerSystemId = 42;
constexpr uint8_t kUpperComputerComponentId = 191;

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

// Motor PWM-monitor format: one targeted PING carries Motor1..Motor4 as four
// uint16_t PWM pulse widths in microseconds in PING.time_usec. On CUAV V6X the
// physical MAIN outputs are reordered into logical motor order before packing.
// SITL has no physical PWM pins and reports a 1000..2000 us equivalent instead.
// PING.seq uses a marker distinct from top-distance reports and includes
// validity, armed state and a frame counter.
constexpr uint32_t kMotorOutputArrayMarkerMask = 0xF0000000u;
constexpr uint32_t kMotorOutputArrayMarker = 0xB0000000u;
constexpr uint32_t kMotorOutputArrayVersionMask = 0x0F000000u;
constexpr uint32_t kMotorOutputArrayVersion = 0x02000000u;
constexpr uint32_t kMotorOutputArrayValidMask = 0x00F00000u;
constexpr uint8_t kMotorOutputArrayValidShift = 20;
constexpr uint32_t kMotorOutputArrayArmedFlag = 0x00010000u;
constexpr uint32_t kMotorOutputArrayReservedMask = 0x000E0000u;
constexpr uint32_t kMotorOutputArraySequenceMask = 0x0000FFFFu;
constexpr uint8_t kMotorOutputArrayValueBits = 16;
constexpr uint16_t kMotorPwmMinimumUs = 500;
constexpr uint16_t kMotorPwmMaximumUs = 2500;
constexpr uint16_t kMotorPwmSimMinimumUs = 1000;
constexpr uint16_t kMotorPwmSimRangeUs = 1000;
constexpr uint8_t kMotorOutputCount = 4;

// Pressure-detail format. PING.time_usec carries four uint16 values:
// requested gain (x10000), applied gain (x10000), time progress (x1000), and
// configured rise time (milliseconds). PING.seq carries the selected trim
// source, available candidates, limiting motor and completion flags.
constexpr uint32_t kPressureDetailMarkerMask = 0xF0000000u;
constexpr uint32_t kPressureDetailMarker = 0xD0000000u;
constexpr uint32_t kPressureDetailVersionMask = 0x0F000000u;
constexpr uint32_t kPressureDetailVersion = 0x01000000u;
constexpr uint32_t kPressureDetailTrimSourceMask = 0x00C00000u;
constexpr uint8_t kPressureDetailTrimSourceShift = 22;
constexpr uint32_t kPressureDetailCandidateMask = 0x00380000u;
constexpr uint8_t kPressureDetailCandidateShift = 19;
constexpr uint32_t kPressureDetailLimitingMotorMask = 0x00070000u;
constexpr uint8_t kPressureDetailLimitingMotorShift = 16;
constexpr uint32_t kPressureDetailLimitedFlag = 0x00008000u;
constexpr uint32_t kPressureDetailCompleteFlag = 0x00004000u;
constexpr uint32_t kPressureDetailValidFlag = 0x00002000u;
constexpr uint32_t kPressureDetailSequenceMask = 0x00001FFFu;
constexpr uint8_t kPressureDetailValueBits = 16;
constexpr float kPressureDetailGainScale = 10000.f;
constexpr float kPressureDetailProgressScale = 1000.f;
constexpr float kPressureDetailTimeScale = 1000.f;

// SEARCH_TOP status format. The controller publishes custom_action_status at
// 5 Hz, and the MAVLink PING stream forwards every update. PING.seq carries
// state/owner/reason plus a small frame counter; PING.time_usec carries the
// current handover id. Repetition makes state display tolerant of WiFi/UDP
// packet loss without adding a custom MAVLink dialect.
constexpr uint32_t kCustomStatusMarkerMask = 0xF0000000u;
constexpr uint32_t kCustomStatusMarker = 0xC0000000u;
constexpr uint32_t kCustomStatusVersionMask = 0x0F000000u;
constexpr uint32_t kCustomStatusVersion = 0x01000000u;
constexpr uint32_t kCustomStatusStateMask = 0x00E00000u;
constexpr uint8_t kCustomStatusStateShift = 21;
constexpr uint32_t kCustomStatusOwnerMask = 0x00180000u;
constexpr uint8_t kCustomStatusOwnerShift = 19;
constexpr uint32_t kCustomStatusActiveFlag = 0x00040000u;
constexpr uint32_t kCustomStatusReasonMask = 0x0003C000u;
constexpr uint8_t kCustomStatusReasonShift = 14;
constexpr uint32_t kCustomStatusSequenceMask = 0x00003FFFu;

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
	StartBusy = 7,
	StartDisabled = 8,
	StartNotArmed = 9,
	StartNotOffboard = 10,
	StartEstimatorInvalid = 11,
	StartSensorInvalid = 12,
	StartDistanceTooClose = 13,
};

constexpr int32_t encodeResult(uint16_t request_id, Result result)
{
	return (static_cast<int32_t>(request_id) << 8) | static_cast<uint8_t>(result);
}
}

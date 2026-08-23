/****************************************************************************
 * Project-local protocol shared by MyLink, custom_action_control and V3.
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace custom_action_protocol
{
constexpr uint16_t kMavCmdUser1 = 31010;
constexpr uint8_t kComponentId = 25; // MAV_COMP_ID_USER1
constexpr uint32_t kTopContactPingContactMask = 1u << 31;
constexpr uint32_t kTopContactPingValidMask = 1u << 30;
constexpr uint32_t kTopContactPingSequenceMask = (1u << 30) - 1u;

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
	TopHoldEntered = 6,
};

constexpr int32_t encodeResult(uint16_t request_id, Result result)
{
	return (static_cast<int32_t>(request_id) << 8) | static_cast<uint8_t>(result);
}
}

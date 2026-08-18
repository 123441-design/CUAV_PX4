/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MylinkBridge.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

#include <commander/px4_custom_mode.h>

#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstring>

using namespace time_literals;

namespace
{
// SIH and the multicopter controller pipeline run at 250 Hz. Direct actuator
// mode replaces ControlAllocator as the actuator_motors publisher, so keep the
// same update rate while active (and use it unconditionally for simplicity).
constexpr hrt_abstime kRunInterval = 4_ms;
constexpr unsigned kMaximumDrainReads = 8;
constexpr float kMotorTestMaximumTimeoutSeconds = 3.f;
constexpr uint8_t kMotorTestThrottlePercent = 0;
constexpr int kDirectMotorCount = 4;
constexpr hrt_abstime kDirectMotorStopHold = 100_ms;
constexpr hrt_abstime kDropLogInterval = 1_s;
constexpr hrt_abstime kHeartbeatInterval = 1_s;
constexpr hrt_abstime kLocalPositionInterval = 200_ms;
constexpr hrt_abstime kBatteryStatusInterval = 1_s;
constexpr hrt_abstime kV4SetpointInterval = 50_ms;
constexpr float kV4MaxHorizontalStep = 0.50f;
constexpr float kV4MaxHorizontalSpeed = 0.20f;
constexpr float kV4MaxAcceleration = 0.40f;
constexpr float kV4PositionTolerance = 0.04f;
constexpr float kV4VelocityTolerance = 0.08f;
constexpr float kV4YawAbortRadians = 0.3490658504f;
constexpr hrt_abstime kV4YawAbortTime = 500_ms;
constexpr uint8_t kV4ActionTakeoff = 1;
constexpr uint8_t kV4ActionUp = 2;
constexpr uint8_t kV4ActionDown = 3;
constexpr uint8_t kV4ActionForward = 4;
constexpr uint8_t kV4ActionBack = 5;
constexpr uint8_t kV4ActionLeft = 6;
constexpr uint8_t kV4ActionRight = 7;
constexpr uint8_t kV4ActionHold = 8;

uint32_t customMode(uint8_t nav_state)
{
	px4_custom_mode mode{};

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	default:
		mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		break;
	}

	return mode.data;
}
}

ModuleBase::Descriptor MylinkBridge::desc{task_spawn, custom_command, print_usage};

MylinkBridge::MylinkBridge(const char *device, uint32_t baudrate) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_serial(device, baudrate)
{
}

MylinkBridge::~MylinkBridge()
{
	ScheduleClear();
	_serial.close();
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MylinkBridge::init()
{
	// NuttX file descriptors belong to the task group that opens them. This
	// module performs I/O on a work queue, so opening the UART here (from the
	// caller/NSH context) would leave Run() with an invalid or unrelated fd.
	// Defer open, configure and all subsequent I/O to the work-queue context.
	ScheduleNow();
	return true;
}

MylinkBridge::GateState MylinkBridge::gateState(vehicle_status_s *status_out)
{
	vehicle_status_s status{};
	const bool available = _vehicle_status_sub.copy(&status) && status.timestamp != 0;

	if (status_out != nullptr) {
		*status_out = status;
	}

	if (!available || status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return GateState::Closed;
	}

	return status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	       ? GateState::Active
	       : GateState::CacheOnly;
}

const char *MylinkBridge::gateStateName(GateState state)
{
	switch (state) {
	case GateState::Closed:
		return "CLOSED";

	case GateState::CacheOnly:
		return "CACHE_ONLY";

	case GateState::Active:
		return "ACTIVE";
	}

	return "UNKNOWN";
}

const char *MylinkBridge::dropReasonName(DropReason reason)
{
	switch (reason) {
	case DropReason::GateClosed:
		return "gate_closed";

	case DropReason::ForbiddenMessage:
		return "forbidden_message";

	case DropReason::ForbiddenCommand:
		return "forbidden_command";

	case DropReason::UnsupportedMessage:
		return "unsupported_message";

	case DropReason::UnsupportedCommand:
		return "unsupported_command";

	case DropReason::InvalidTarget:
		return "invalid_target";

	case DropReason::InvalidPayload:
		return "invalid_payload";
	}

	return "unknown";
}

const char *MylinkBridge::messageName(uint32_t message_id)
{
	switch (message_id) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		return "HEARTBEAT";

	case MAVLINK_MSG_ID_PING:
		return "PING";

	case MAVLINK_MSG_ID_COMMAND_LONG:
		return "COMMAND_LONG";

	case MAVLINK_MSG_ID_COMMAND_INT:
		return "COMMAND_INT";

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		return "SET_POSITION_TARGET_LOCAL_NED";

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		return "MANUAL_CONTROL";

	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		return "RC_CHANNELS_OVERRIDE";

	default:
		return "UNKNOWN";
	}
}

void MylinkBridge::updateGateStateLog(GateState state, const vehicle_status_s &status)
{
	const bool armed = status.timestamp != 0
			   && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (!_gate_state_initialized) {
		PX4_INFO("gate initial=%s armed=%s nav_state=%u", gateStateName(state), armed ? "yes" : "no", status.nav_state);
		_last_gate_state = state;
		_gate_state_initialized = true;
		return;
	}

	if (state != _last_gate_state) {
		PX4_INFO("gate %s -> %s armed=%s nav_state=%u", gateStateName(_last_gate_state), gateStateName(state),
			 armed ? "yes" : "no", status.nav_state);
		_last_gate_state = state;
	}
}

void MylinkBridge::recordDrop(const mavlink_message_t &message, GateState state, DropReason reason, uint16_t command)
{
	const hrt_abstime now = hrt_absolute_time();
	const bool changed = message.msgid != _last_drop_message_id
			     || command != _last_drop_command
			     || state != _last_drop_state
			     || reason != _last_drop_reason;

	_gate_dropped_frames++;

	_last_drop_message_id = message.msgid;
	_last_drop_command = command;
	_last_drop_state = state;
	_last_drop_reason = reason;
	_last_drop_timestamp = now;

	if (changed || _last_drop_log_timestamp == 0 || now - _last_drop_log_timestamp >= kDropLogInterval) {
		if (command != 0) {
			PX4_WARN("drop gate=%s msg=%s(%u) cmd=%u reason=%s repeated=%" PRIu32,
				 gateStateName(state), messageName(message.msgid), static_cast<unsigned>(message.msgid),
				 static_cast<unsigned>(command),
				 dropReasonName(reason), _suppressed_drop_logs);

		} else {
			PX4_WARN("drop gate=%s msg=%s(%u) reason=%s repeated=%" PRIu32,
				 gateStateName(state), messageName(message.msgid), static_cast<unsigned>(message.msgid), dropReasonName(reason),
				 _suppressed_drop_logs);
		}

		_last_drop_log_timestamp = now;
		_suppressed_drop_logs = 0;

	} else {
		_suppressed_drop_logs++;
	}
}

bool MylinkBridge::targetOk(uint8_t target_system, uint8_t target_component)
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);

	return (target_system == 0 || target_system == system_id)
	       && (target_component == MAV_COMP_ID_ALL || target_component == component_id);
}

bool MylinkBridge::commandSupported(uint16_t command) const
{
	switch (command) {
	case MAV_CMD_USER_1:
	case MAV_CMD_NAV_RETURN_TO_LAUNCH:
	case MAV_CMD_NAV_LAND:
	case MAV_CMD_NAV_TAKEOFF:
	case MAV_CMD_DO_CHANGE_SPEED:
	case MAV_CMD_DO_PAUSE_CONTINUE:
	case MAV_CMD_MISSION_START:
	case MAV_CMD_DO_MOTOR_TEST:
		return true;

	default:
		return false;
	}
}

uint32_t MylinkBridge::eventFlagForCommand(const mavlink_command_long_t &command) const
{
	switch (command.command) {
	case MAV_CMD_NAV_TAKEOFF:
		return EventTakeoff;

	case MAV_CMD_NAV_LAND:
		return EventLand;

	case MAV_CMD_DO_CHANGE_SPEED:
		return EventSpeed;

	case MAV_CMD_DO_PAUSE_CONTINUE:
		return command.param1 > 0.5f ? EventContinue : EventPause;

	case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		return EventRtl;

	case MAV_CMD_MISSION_START:
		return EventMissionStart;

	case MAV_CMD_DO_MOTOR_TEST:
		return EventMotorThrottle;

	default:
		return EventNone;
	}
}

void MylinkBridge::sendMavlinkMessage(const mavlink_message_t &message)
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
	errno = 0;
	const ssize_t written = _serial.writeBlocking(buffer, length, 10);

	if (written > 0) {
		_tx_bytes += static_cast<uint32_t>(written);
	}

	if (written == static_cast<ssize_t>(length)) {
		_tx_frames++;

	} else {
		_tx_errors++;
		_last_tx_errno = (written < 0 && errno != 0) ? errno : EIO;
		PX4_WARN("MAVLink TX incomplete: %zd/%u errno=%d", written, length, _last_tx_errno);
	}
}

void MylinkBridge::sendCommandAck(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2,
				uint8_t target_system, uint8_t target_component)
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);

	mavlink_message_t response{};
	mavlink_msg_command_ack_pack_status(system_id, component_id, &_tx_status, &response,
					    command, result, progress, result_param2,
					    target_system, target_component);
	sendMavlinkMessage(response);
	if (command == MAV_CMD_USER_1) {
		_v4_ack_sent++;
	}
}

void MylinkBridge::handlePing(const mavlink_message_t &message)
{
	mavlink_ping_t ping{};
	mavlink_msg_ping_decode(&message, &ping);

	// target 0/0 is the standard MAVLink ping request. A targeted PING is a
	// response and must not be echoed again.
	if (ping.target_system != 0 || ping.target_component != 0) {
		return;
	}

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);

	mavlink_message_t response{};
	mavlink_msg_ping_pack_status(system_id, component_id, &_tx_status, &response,
				     ping.time_usec, ping.seq, message.sysid, message.compid);
	sendMavlinkMessage(response);
	_handled_messages++;
}

void MylinkBridge::handleHeartbeat(const mavlink_message_t &message)
{
	_peer_system_id = message.sysid;
	_peer_component_id = message.compid;
	_handled_messages++;
}

void MylinkBridge::publishVehicleCommand(const mavlink_message_t &message,
		const mavlink_command_long_t &command, uint16_t vehicle_command_id)
{
	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.param1 = command.param1;
	vehicle_command.param2 = command.param2;
	vehicle_command.param3 = command.param3;
	vehicle_command.param4 = command.param4;
	vehicle_command.param5 = command.param5;
	vehicle_command.param6 = command.param6;
	vehicle_command.param7 = command.param7;
	vehicle_command.command = vehicle_command_id;
	vehicle_command.target_system = command.target_system;
	vehicle_command.target_component = command.target_component;
	vehicle_command.source_system = message.sysid;
	vehicle_command.source_component = message.compid;
	vehicle_command.confirmation = command.confirmation;
	vehicle_command.from_external = true;
	_vehicle_command_pub.publish(vehicle_command);
	_peer_system_id = message.sysid;
	_peer_component_id = message.compid;
	_handled_messages++;
}

float MylinkBridge::wrapPi(float angle)
{
	while (angle > 3.14159265359f) {
		angle -= 6.28318530718f;
	}

	while (angle < -3.14159265359f) {
		angle += 6.28318530718f;
	}

	return angle;
}

bool MylinkBridge::v4HeadingValid(const vehicle_local_position_s &position) const
{
	return position.heading_good_for_control && PX4_ISFINITE(position.heading);
}

bool MylinkBridge::v4Busy() const
{
	return _v4_action_state != V4ActionState::Hold && _v4_action_state != V4ActionState::Landing;
}

void MylinkBridge::captureV4Hold(const vehicle_local_position_s &position, bool keep_heading)
{
	_v4_initialized = true;
	_v4_action_state = V4ActionState::Hold;
	_v4_target_position[0] = position.x;
	_v4_target_position[1] = position.y;
	_v4_target_position[2] = position.z;
	_v4_command_position[0] = position.x;
	_v4_command_position[1] = position.y;
	_v4_command_position[2] = position.z;
	_v4_command_velocity[0] = 0.f;
	_v4_command_velocity[1] = 0.f;
	_v4_command_velocity[2] = 0.f;
	_v4_yaw_ref = keep_heading && v4HeadingValid(position) ? position.heading : NAN;
	_v4_heading_reset_ref = position.heading_reset_counter;
	_v4_bad_yaw_since = 0;
}

void MylinkBridge::cancelV4ToHold(const vehicle_local_position_s &position,
				 bool count_heading_abort, bool count_heading_reset)
{
	if (count_heading_abort) {
		_v4_heading_abort++;
	}

	if (count_heading_reset) {
		_v4_heading_reset_abort++;
	}

	captureV4Hold(position, v4HeadingValid(position));
}

void MylinkBridge::handleV4UserCommand(const mavlink_message_t &message, GateState state,
				       const mavlink_command_long_t &command)
{
	_v4_rx_command++;

	auto reject = [&](uint8_t result) {
		_v4_rejected++;
		sendCommandAck(MAV_CMD_USER_1, result, UINT8_MAX, 0, message.sysid, message.compid);
	};

	if (state != GateState::Active) {
		reject(MAV_RESULT_TEMPORARILY_REJECTED);
		return;
	}

	if (!PX4_ISFINITE(command.param1) || fabsf(command.param1 - roundf(command.param1)) > 0.001f
	    || command.param1 < kV4ActionTakeoff || command.param1 > kV4ActionHold) {
		reject(MAV_RESULT_DENIED);
		return;
	}

	const uint8_t action = static_cast<uint8_t>(lroundf(command.param1));
	vehicle_local_position_s position{};
	if (!_vehicle_local_position_sub.copy(&position) || position.timestamp == 0
	    || !PX4_ISFINITE(position.x) || !PX4_ISFINITE(position.y) || !PX4_ISFINITE(position.z)) {
		reject(MAV_RESULT_TEMPORARILY_REJECTED);
		return;
	}

	if (action == kV4ActionHold) {
		captureV4Hold(position, true);
		_setpoint_cache_valid = false;
		_v4_hold_count++;
		_v4_accepted++;
		sendCommandAck(MAV_CMD_USER_1, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, message.sysid, message.compid);
		return;
	}

	if (_v4_action_state == V4ActionState::Landing) {
		_v4_busy_rejected++;
		reject(MAV_RESULT_TEMPORARILY_REJECTED);
		return;
	}

	if (v4Busy()) {
		_v4_busy_rejected++;
		reject(MAV_RESULT_TEMPORARILY_REJECTED);
		return;
	}

	if (!PX4_ISFINITE(command.param2) || command.param2 <= 0.f) {
		reject(MAV_RESULT_DENIED);
		return;
	}

	const bool horizontal = action >= kV4ActionForward && action <= kV4ActionRight;
	if (horizontal && !v4HeadingValid(position)) {
		_v4_heading_invalid_reject++;
		reject(MAV_RESULT_TEMPORARILY_REJECTED);
		return;
	}

	float amount = command.param2;
	if (horizontal) {
		amount = math::min(amount, kV4MaxHorizontalStep);
	}

	_v4_initialized = true;
	_v4_action_state = action == kV4ActionTakeoff ? V4ActionState::Takeoff
				 : horizontal ? V4ActionState::HorizontalMove : V4ActionState::VerticalMove;
	_v4_target_position[0] = position.x;
	_v4_target_position[1] = position.y;
	_v4_target_position[2] = position.z;
	_v4_command_position[0] = position.x;
	_v4_command_position[1] = position.y;
	_v4_command_position[2] = position.z;
	_v4_command_velocity[0] = 0.f;
	_v4_command_velocity[1] = 0.f;
	_v4_command_velocity[2] = 0.f;
	_v4_yaw_ref = v4HeadingValid(position) ? position.heading : NAN;
	_v4_heading_reset_ref = position.heading_reset_counter;
	_v4_bad_yaw_since = 0;
	_setpoint_cache_valid = false;

	if (action == kV4ActionTakeoff) {
		_v4_target_position[2] = position.z - amount;
		_v4_takeoff_count++;

	} else if (horizontal) {
		const float heading = position.heading;
		const float c = cosf(heading);
		const float s = sinf(heading);
		float dx = 0.f;
		float dy = 0.f;

		switch (action) {
		case kV4ActionForward:
			dx = amount * c; dy = amount * s; break;
		case kV4ActionBack:
			dx = -amount * c; dy = -amount * s; break;
		case kV4ActionRight:
			dx = -amount * s; dy = amount * c; break;
		case kV4ActionLeft:
			dx = amount * s; dy = -amount * c; break;
		default:
			break;
		}

		_v4_target_position[0] += dx;
		_v4_target_position[1] += dy;
		_v4_move_count++;

	} else {
		_v4_target_position[2] += action == kV4ActionUp ? -amount : amount;
	}

	_v4_accepted++;
	sendCommandAck(MAV_CMD_USER_1, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, message.sysid, message.compid);
}

void MylinkBridge::handleCommandLong(const mavlink_message_t &message, GateState state)
{
	mavlink_command_long_t command{};
	mavlink_msg_command_long_decode(&message, &command);

	if (!targetOk(command.target_system, command.target_component)) {
		recordDrop(message, state, DropReason::InvalidTarget, command.command);
		return;
	}

	if (command.command == MAV_CMD_USER_1) {
		handleV4UserCommand(message, state, command);
		return;
	}

	if (command.command == MAV_CMD_DO_SET_MODE) {
		const uint8_t base_mode = static_cast<uint8_t>(command.param1);
		const uint8_t custom_main_mode = static_cast<uint8_t>(command.param2);
		const bool requests_offboard = (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
					       && custom_main_mode == 6;

		if (requests_offboard) {
			handleOffboardModeCommand(message, command);

		} else {
			sendCommandAck(command.command, MAV_RESULT_DENIED, UINT8_MAX, 0, message.sysid, message.compid);
			recordDrop(message, state, DropReason::ForbiddenCommand, command.command);
		}

		return;
	}

	if (command.command == MAV_CMD_NAV_LAND) {
		_v4_action_state = V4ActionState::Landing;
		_setpoint_cache_valid = false;
		_v4_land_count++;
		handleLandCommand(message, command);
		return;
	}

	if (command.command == MAV_CMD_COMPONENT_ARM_DISARM) {
		publishVehicleCommand(message, command, vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM);
		return;
	}

	if (command.command == MAV_CMD_NAV_TAKEOFF) {
		publishVehicleCommand(message, command, vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
		return;
	}

	if (command.command == MAV_CMD_DO_MOTOR_TEST || command.command == MAV_CMD_DO_SET_ACTUATOR) {
		sendCommandAck(command.command, MAV_RESULT_DENIED, UINT8_MAX, 0, message.sysid, message.compid);
		recordDrop(message, state, DropReason::ForbiddenCommand, command.command);
		return;
	}

	sendCommandAck(command.command, MAV_RESULT_UNSUPPORTED, UINT8_MAX, 0, message.sysid, message.compid);
	_unsupported_messages++;
	recordDrop(message, state, DropReason::UnsupportedCommand, command.command);
}

void MylinkBridge::handleMotorTestCommand(const mavlink_message_t &message,
		const mavlink_command_long_t &command)
{
	const int motor_instance = static_cast<int>(lroundf(command.param1));
	const int throttle_type = static_cast<int>(lroundf(command.param2));
	const int motor_count = static_cast<int>(lroundf(command.param5));
	const bool integer_fields_valid = PX4_ISFINITE(command.param1) && PX4_ISFINITE(command.param2)
					  && PX4_ISFINITE(command.param5)
					  && fabsf(command.param1 - motor_instance) < 0.001f
					  && fabsf(command.param2 - throttle_type) < 0.001f
					  && fabsf(command.param5 - motor_count) < 0.001f;
	const bool valid = integer_fields_valid
			   && motor_instance == 1
			   && throttle_type == kMotorTestThrottlePercent
			   && PX4_ISFINITE(command.param3) && command.param3 >= 0.f && command.param3 <= 100.f
			   && PX4_ISFINITE(command.param4) && command.param4 > 0.f
			   && command.param4 <= kMotorTestMaximumTimeoutSeconds
			   && motor_count == kDirectMotorCount;

	if (!valid) {
		_invalid_motor_test_commands++;
		sendCommandAck(command.command, MAV_RESULT_DENIED, UINT8_MAX, 0, message.sysid, message.compid);
		return;
	}

	const uint32_t timeout_ms = static_cast<uint32_t>(lroundf(command.param4 * 1000.f));
	activateDirectMotorControl(command.param3, timeout_ms);
	_latest_command = command;
	_latest_event_command = command.command;
	_event_flags |= EventMotorThrottle;
	_motor_test_commands++;
	_handled_messages++;
	sendCommandAck(command.command, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, message.sysid, message.compid);
}

void MylinkBridge::activateDirectMotorControl(float throttle_percent, uint32_t timeout_ms)
{
	const hrt_abstime now = hrt_absolute_time();
	_direct_motor_active = true;
	_direct_motor_stopping = false;
	_motor_throttle_percent = throttle_percent;
	_motor_test_deadline = now + timeout_ms * 1000ULL;
	_motor_stop_deadline = 0;
}

void MylinkBridge::publishDirectMotorSetpoint(float throttle_percent)
{
	const hrt_abstime now = hrt_absolute_time();
	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = now;
	motors.reversible_flags = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		motors.control[i] = i < kDirectMotorCount ? throttle_percent * 0.01f : NAN;
	}

	_actuator_motors_pub.publish(motors);
	_direct_motor_setpoints_published++;
}

void MylinkBridge::updateDirectMotorControl(GateState state)
{
	if (!_direct_motor_active && !_direct_motor_stopping) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (state != GateState::Active) {
		releaseDirectMotorControl(false);
		return;
	}

	if (_direct_motor_active && now >= _motor_test_deadline) {
		releaseDirectMotorControl();
	}

	if (_direct_motor_stopping && now >= _motor_stop_deadline) {
		_direct_motor_stopping = false;
		_motor_stop_deadline = 0;
		return;
	}

	// Commander consumes this asynchronously. Publish the direct-actuator
	// control type for the full command lifetime, not only on command receipt.
	offboard_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.direct_actuator = true;
	_offboard_control_mode_pub.publish(control_mode);
	_offboard_control_mode_published++;

	vehicle_control_mode_s vehicle_control_mode{};
	const bool control_mode_available = _vehicle_control_mode_sub.copy(&vehicle_control_mode)
					    && vehicle_control_mode.timestamp != 0;
	const bool direct_actuator_ready = control_mode_available
					   && vehicle_control_mode.flag_armed
					   && vehicle_control_mode.flag_control_offboard_enabled
					   && !vehicle_control_mode.flag_control_allocation_enabled;

	// ControlAllocator is another actuator_motors publisher. Wait until
	// Commander has disabled it before publishing the four direct values.
	if (direct_actuator_ready) {
		publishDirectMotorSetpoint(_direct_motor_stopping ? 0.f : _motor_throttle_percent);
	}
}

void MylinkBridge::releaseDirectMotorControl(bool hold_zero)
{
	if (!_direct_motor_active && !_direct_motor_stopping) {
		return;
	}
	const bool command_was_active = _direct_motor_active;

	_direct_motor_active = false;
	_direct_motor_stopping = hold_zero;
	_motor_throttle_percent = 0.f;
	_motor_test_deadline = 0;
	_motor_stop_deadline = hold_zero ? hrt_absolute_time() + kDirectMotorStopHold : 0;

	if (command_was_active) {
		_motor_test_releases++;
	}

	// Zero is an explicit minimum-throttle command. Keep publishing it briefly
	// before allowing the velocity stream to return control to ControlAllocator.
	publishDirectMotorSetpoint(0.f);
}

void MylinkBridge::handleOffboardModeCommand(const mavlink_message_t &message,
		const mavlink_command_long_t &command)
{
	constexpr uint8_t kCustomModeEnabled = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	constexpr uint8_t kPx4CustomMainModeOffboard = 6;
	const uint8_t base_mode = static_cast<uint8_t>(command.param1);
	const uint8_t custom_main_mode = static_cast<uint8_t>(command.param2);

	// This bridge only grants a serial request to enter Offboard. It cannot arm
	// the aircraft or use this path to switch to any other flight mode.
	if (!(base_mode & kCustomModeEnabled) || custom_main_mode != kPx4CustomMainModeOffboard) {
		sendCommandAck(command.command, MAV_RESULT_DENIED, UINT8_MAX, 0, message.sysid, message.compid);
		_unsupported_messages++;
		return;
	}

	publishVehicleCommand(message, command, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE);
	_offboard_mode_requests++;
}

void MylinkBridge::handleLandCommand(const mavlink_message_t &message,
				    const mavlink_command_long_t &command)
{
	publishVehicleCommand(message, command, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
}

void MylinkBridge::handleCommandInt(const mavlink_message_t &message, GateState state)
{
	mavlink_command_int_t command{};
	mavlink_msg_command_int_decode(&message, &command);

	if (!targetOk(command.target_system, command.target_component)) {
		recordDrop(message, state, DropReason::InvalidTarget, command.command);
		return;
	}

	if (command.command == MAV_CMD_NAV_LAND) {
		vehicle_command_s vehicle_command{};
		vehicle_command.timestamp = hrt_absolute_time();
		vehicle_command.param1 = command.param1;
		vehicle_command.param2 = command.param2;
		vehicle_command.param3 = command.param3;
		vehicle_command.param4 = command.param4;
		vehicle_command.param5 = static_cast<double>(command.x) * 1e-7;
		vehicle_command.param6 = static_cast<double>(command.y) * 1e-7;
		vehicle_command.param7 = command.z;
		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
		vehicle_command.target_system = command.target_system;
		vehicle_command.target_component = command.target_component;
		vehicle_command.source_system = message.sysid;
		vehicle_command.source_component = message.compid;
		vehicle_command.confirmation = false;
		vehicle_command.from_external = true;
		_vehicle_command_pub.publish(vehicle_command);
		_peer_system_id = message.sysid;
		_peer_component_id = message.compid;
		_handled_messages++;
		return;
	}

	if (command.command == MAV_CMD_DO_MOTOR_TEST || command.command == MAV_CMD_DO_SET_ACTUATOR) {
		sendCommandAck(command.command, MAV_RESULT_DENIED, UINT8_MAX, 0, message.sysid, message.compid);
		recordDrop(message, state, DropReason::ForbiddenCommand, command.command);
		return;
	}

	sendCommandAck(command.command, MAV_RESULT_COMMAND_LONG_ONLY, UINT8_MAX, 0,
		       message.sysid, message.compid);
	_unsupported_messages++;
	recordDrop(message, state, DropReason::UnsupportedCommand, command.command);
}

bool MylinkBridge::decodeLocalNedSetpoint(const mavlink_message_t &message,
		trajectory_setpoint_s &setpoint, offboard_control_mode_s &control_mode)
{
	mavlink_set_position_target_local_ned_t target{};
	mavlink_msg_set_position_target_local_ned_decode(&message, &target);

	if (!targetOk(target.target_system, target.target_component)
	    || target.coordinate_frame != MAV_FRAME_LOCAL_NED) {
		return false;
	}

	const uint16_t mask = target.type_mask;
	setpoint = {};

	for (unsigned i = 0; i < 3; ++i) {
		setpoint.position[i] = NAN;
		setpoint.velocity[i] = NAN;
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
	}

	setpoint.position[0] = (mask & POSITION_TARGET_TYPEMASK_X_IGNORE) ? NAN : target.x;
	setpoint.position[1] = (mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) ? NAN : target.y;
	setpoint.position[2] = (mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) ? NAN : target.z;
	setpoint.velocity[0] = (mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? NAN : target.vx;
	setpoint.velocity[1] = (mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? NAN : target.vy;
	setpoint.velocity[2] = (mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? NAN : target.vz;
	setpoint.acceleration[0] = (mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? NAN : target.afx;
	setpoint.acceleration[1] = (mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? NAN : target.afy;
	setpoint.acceleration[2] = (mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? NAN : target.afz;
	setpoint.yaw = (mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE) ? NAN : target.yaw;
	setpoint.yawspeed = (mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) ? NAN : target.yaw_rate;

	control_mode = {};
	control_mode.position = PX4_ISFINITE(setpoint.position[0]) || PX4_ISFINITE(setpoint.position[1])
				|| PX4_ISFINITE(setpoint.position[2]);
	control_mode.velocity = PX4_ISFINITE(setpoint.velocity[0]) || PX4_ISFINITE(setpoint.velocity[1])
				|| PX4_ISFINITE(setpoint.velocity[2]);
	control_mode.acceleration = PX4_ISFINITE(setpoint.acceleration[0]) || PX4_ISFINITE(setpoint.acceleration[1])
				    || PX4_ISFINITE(setpoint.acceleration[2]);
	control_mode.attitude = PX4_ISFINITE(setpoint.yaw);
	control_mode.body_rate = PX4_ISFINITE(setpoint.yawspeed);

	if ((control_mode.acceleration && (mask & POSITION_TARGET_TYPEMASK_FORCE_SET))
	    || !(control_mode.position || control_mode.velocity || control_mode.acceleration)) {
		return false;
	}

	return true;
}

void MylinkBridge::updateV4Setpoint(GateState state)
{
	if (state == GateState::Closed || _setpoint_cache_valid) {
		return;
	}

	vehicle_local_position_s actual{};
	if (!_vehicle_local_position_sub.copy(&actual) || actual.timestamp == 0
	    || !PX4_ISFINITE(actual.x) || !PX4_ISFINITE(actual.y) || !PX4_ISFINITE(actual.z)) {
		return;
	}

	if (!_v4_initialized) {
		captureV4Hold(actual, true);
	}

	if (state != GateState::Active && v4Busy()) {
		captureV4Hold(actual, v4HeadingValid(actual));
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = _v4_last_setpoint == 0 ? 0.05f
			 : math::constrain(static_cast<float>(now - _v4_last_setpoint) * 1e-6f, 0.01f, 0.10f);
	_v4_last_setpoint = now;

	if (v4Busy()) {
		if (actual.heading_reset_counter != _v4_heading_reset_ref) {
			cancelV4ToHold(actual, false, true);

		} else if (!v4HeadingValid(actual)) {
			cancelV4ToHold(actual, true, false);

		} else if (PX4_ISFINITE(_v4_yaw_ref)) {
			const float yaw_error = wrapPi(actual.heading - _v4_yaw_ref);

			if (fabsf(yaw_error) > kV4YawAbortRadians) {
				if (_v4_bad_yaw_since == 0) {
					_v4_bad_yaw_since = now;
				}

				if (now - _v4_bad_yaw_since >= kV4YawAbortTime) {
					cancelV4ToHold(actual, true, false);
				}

			} else {
				_v4_bad_yaw_since = 0;
			}
		}
	}

	if (_v4_action_state == V4ActionState::Takeoff) {
		_v4_command_position[0] = _v4_target_position[0];
		_v4_command_position[1] = _v4_target_position[1];
		_v4_command_position[2] = _v4_target_position[2];
		_v4_command_velocity[0] = 0.f;
		_v4_command_velocity[1] = 0.f;
		_v4_command_velocity[2] = 0.f;

		if (fabsf(actual.z - _v4_target_position[2]) <= kV4PositionTolerance
		    && fabsf(actual.vz) <= kV4VelocityTolerance) {
			_v4_action_state = V4ActionState::Hold;
		}

	} else if (_v4_action_state == V4ActionState::HorizontalMove
		   || _v4_action_state == V4ActionState::VerticalMove) {
		const bool horizontal = _v4_action_state == V4ActionState::HorizontalMove;
		const float ex = _v4_target_position[0] - _v4_command_position[0];
		const float ey = _v4_target_position[1] - _v4_command_position[1];
		const float ez = _v4_target_position[2] - _v4_command_position[2];
		const float distance = horizontal ? hypotf(ex, ey) : fabsf(ez);

		if (distance <= 1e-4f) {
			_v4_command_position[0] = _v4_target_position[0];
			_v4_command_position[1] = _v4_target_position[1];
			_v4_command_position[2] = _v4_target_position[2];
			_v4_command_velocity[0] = 0.f;
			_v4_command_velocity[1] = 0.f;
			_v4_command_velocity[2] = 0.f;
			_v4_action_state = V4ActionState::Hold;

		} else {
			const float desired_speed = math::min(kV4MaxHorizontalSpeed,
					 sqrtf(fmaxf(0.f, 2.f * kV4MaxAcceleration * distance)));
			const float step = math::min(distance, desired_speed * dt);

			if (horizontal) {
				const float scale = step / distance;
				_v4_command_position[0] += ex * scale;
				_v4_command_position[1] += ey * scale;
				_v4_command_velocity[0] = ex / distance * desired_speed;
				_v4_command_velocity[1] = ey / distance * desired_speed;
				_v4_command_velocity[2] = 0.f;

			} else {
				_v4_command_position[2] += ez / distance * step;
				_v4_command_velocity[0] = 0.f;
				_v4_command_velocity[1] = 0.f;
				_v4_command_velocity[2] = ez / distance * desired_speed;
			}

			if (step >= distance - 1e-5f) {
				_v4_command_position[0] = _v4_target_position[0];
				_v4_command_position[1] = _v4_target_position[1];
				_v4_command_position[2] = _v4_target_position[2];
				_v4_command_velocity[0] = 0.f;
				_v4_command_velocity[1] = 0.f;
				_v4_command_velocity[2] = 0.f;
				_v4_action_state = V4ActionState::Hold;
			}
		}
	}

	trajectory_setpoint_s setpoint{};
	for (unsigned i = 0; i < 3; ++i) {
		setpoint.position[i] = _v4_command_position[i];
		setpoint.velocity[i] = NAN;
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
	}

	const bool use_velocity = _v4_action_state == V4ActionState::HorizontalMove
				 || _v4_action_state == V4ActionState::VerticalMove;
	if (use_velocity) {
		for (unsigned i = 0; i < 3; ++i) {
			setpoint.velocity[i] = _v4_command_velocity[i];
		}
	}

	setpoint.yaw = PX4_ISFINITE(_v4_yaw_ref) ? _v4_yaw_ref : NAN;
	setpoint.yawspeed = NAN;
	setpoint.timestamp = now;

	offboard_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.position = true;
	control_mode.velocity = use_velocity;
	control_mode.acceleration = false;
	control_mode.attitude = PX4_ISFINITE(setpoint.yaw);
	control_mode.body_rate = false;
	_offboard_control_mode_pub.publish(control_mode);
	_offboard_control_mode_published++;
	_trajectory_setpoint_pub.publish(setpoint);
	_trajectory_setpoints_published++;
}

void MylinkBridge::handleSetPositionTargetLocalNed(const mavlink_message_t &message, GateState state)
{
	// Direct motor control owns the Offboard control type while it is active.
	// A parallel velocity stream must not switch control allocation back on.
	if (_direct_motor_active || _direct_motor_stopping) {
		return;
	}

	trajectory_setpoint_s setpoint{};
	offboard_control_mode_s control_mode{};

	if (!decodeLocalNedSetpoint(message, setpoint, control_mode)) {
		_invalid_setpoints++;
		recordDrop(message, state, DropReason::InvalidPayload);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	_offboard_setpoints_received++;
	_last_setpoint_rx = now;
	_latest_setpoint = setpoint;
	_setpoint_cache_valid = true;

	// Every fresh external setpoint is an Offboard availability heartbeat.
	// Never periodically republish this from the cache: a stopped serial stream
	// must allow PX4's normal Offboard-loss timeout to take effect.
	control_mode.timestamp = now;
	_offboard_control_mode_pub.publish(control_mode);
	_offboard_control_mode_published++;

	// Before Offboard this message only prewarms the mode. Flight setpoints are
	// published after the gate is ACTIVE.
	if (state == GateState::Active) {
		setpoint.timestamp = now;
		_trajectory_setpoint_pub.publish(setpoint);
		_trajectory_setpoints_published++;
	}
}

bool MylinkBridge::cacheableMessage(const mavlink_message_t &message) const
{
	// Cache only actionable control commands. Diagnostic PING messages must
	// never overwrite a command waiting for Offboard activation.
	return message.msgid == MAVLINK_MSG_ID_COMMAND_LONG;
}

void MylinkBridge::cacheMessage(const mavlink_message_t &message)
{
	_cached_message = message;
	_cache_valid = true;
	_cached_updates++;
}

void MylinkBridge::clearSessionState()
{
	_direct_motor_active = false;
	_direct_motor_stopping = false;
	_motor_throttle_percent = 0.f;
	_cached_message = {};
	_latest_command = {};
	_latest_setpoint = {};
	_cache_valid = false;
	_setpoint_cache_valid = false;
	_last_setpoint_rx = 0;
	_event_flags = EventNone;
	_latest_event_command = 0;
	_v4_initialized = false;
	_v4_action_state = V4ActionState::Hold;
	_v4_yaw_ref = NAN;
	_v4_bad_yaw_since = 0;
	_v4_last_setpoint = 0;
	_session_resets++;
}

void MylinkBridge::processMavlinkMessage(const mavlink_message_t &message)
{
	handleMavlinkMessage(message);
}

void MylinkBridge::handleMavlinkMessage(const mavlink_message_t &message)
{
	const GateState state = gateState();

	switch (message.msgid) {
	case MAVLINK_MSG_ID_PING:
		handlePing(message);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handleHeartbeat(message);
		break;

	case MAVLINK_MSG_ID_COMMAND_LONG:
		handleCommandLong(message, state);
		break;

	case MAVLINK_MSG_ID_COMMAND_INT:
		handleCommandInt(message, state);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		if (state == GateState::Closed) {
			recordDrop(message, state, DropReason::GateClosed);

		} else {
			handleSetPositionTargetLocalNed(message, state);
		}

		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		recordDrop(message, state, DropReason::ForbiddenMessage);
		break;

	default:
		_unsupported_messages++;
		recordDrop(message, state, DropReason::UnsupportedMessage);
		break;
	}
}

void MylinkBridge::updateGateAndCachedMessage()
{
	vehicle_status_s status{};
	const GateState state = gateState(&status);
	const bool armed = status.timestamp != 0
			   && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const GateState previous_state = _last_gate_state;

	updateGateStateLog(state, status);

	if (previous_state == GateState::Active && state != GateState::Active && _v4_initialized && v4Busy()) {
		vehicle_local_position_s position{};
		if (_vehicle_local_position_sub.copy(&position) && position.timestamp != 0) {
			captureV4Hold(position, v4HeadingValid(position));
		}
	}

	if (_was_armed && !armed) {
		clearSessionState();
	}

	_was_armed = armed;

	// V3 does not replay control commands after a gate transition. Keep the
	// legacy storage members for rollback, but discard any pending entry.
	if (_cache_valid) {
		_cached_message = {};
		_cache_valid = false;
	}
}

void MylinkBridge::forwardCommandAcks()
{
	vehicle_command_ack_s ack{};

	while (_vehicle_command_ack_sub.update(&ack)) {
		if (ack.from_external || _peer_system_id == 0
		    || ack.target_system != _peer_system_id
		    || ack.target_component != _peer_component_id
		    || ack.target_component > UINT8_MAX) {
			continue;
		}

		sendCommandAck(static_cast<uint16_t>(ack.command), ack.result, ack.result_param1,
			       ack.result_param2, ack.target_system, static_cast<uint8_t>(ack.target_component));
		_command_acks_forwarded++;
	}
}

void MylinkBridge::sendHeartbeat()
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
	}

	uint8_t vehicle_type = MAV_TYPE_GENERIC;

	if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		vehicle_type = MAV_TYPE_QUADROTOR;

	} else if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		vehicle_type = MAV_TYPE_FIXED_WING;

	} else if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		vehicle_type = MAV_TYPE_GROUND_ROVER;
	}

	const uint8_t system_state = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
				     ? (status.failsafe ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE)
				     : (status.pre_flight_checks_pass ? MAV_STATE_STANDBY : MAV_STATE_UNINIT);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id
				     : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);
	mavlink_message_t message{};
	mavlink_msg_heartbeat_pack_status(system_id, component_id, &_tx_status, &message,
					 vehicle_type, MAV_AUTOPILOT_PX4, base_mode,
					 customMode(status.nav_state_display), system_state);
	sendMavlinkMessage(message);
	_heartbeats_sent++;
}

void MylinkBridge::sendLocalPositionNed()
{
	vehicle_local_position_s position{};

	if (!_vehicle_local_position_sub.copy(&position) || position.timestamp == 0) {
		return;
	}

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id
				     : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);
	mavlink_message_t message{};
	mavlink_msg_local_position_ned_pack_status(system_id, component_id, &_tx_status, &message,
						  static_cast<uint32_t>(position.timestamp / 1000),
						  position.x, position.y, position.z, position.vx, position.vy, position.vz);
	sendMavlinkMessage(message);
	_local_positions_sent++;
}

void MylinkBridge::sendBatteryStatus()
{
	battery_status_s battery{};

	if (!_battery_status_sub.copy(&battery) || battery.timestamp == 0) {
		return;
	}

	uint16_t voltages[10];

	for (uint16_t &voltage : voltages) {
		voltage = UINT16_MAX;
	}

	if (battery.connected && battery.voltage_v > 0.f) {
		voltages[0] = battery.voltage_v < 65.534f
			      ? static_cast<uint16_t>(battery.voltage_v * 1000.f) : UINT16_MAX - 1;
	}

	const int16_t current = battery.connected && battery.current_a >= 0.f
				? (battery.current_a < 327.67f ? static_cast<int16_t>(battery.current_a * 100.f) : INT16_MAX)
				: -1;
	const int8_t remaining = battery.connected && battery.remaining >= 0.f
				 ? (battery.remaining < 1.f ? static_cast<int8_t>(battery.remaining * 100.f) : 100) : -1;
	uint16_t voltages_ext[4]{};
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id
				     : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);
	mavlink_message_t message{};
	mavlink_msg_battery_status_pack_status(system_id, component_id, &_tx_status, &message,
					      battery.id > 0 ? battery.id - 1 : 0,
					      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX, voltages,
					      current, -1, -1, remaining, 0, MAV_BATTERY_CHARGE_STATE_UNDEFINED,
					      voltages_ext, MAV_BATTERY_MODE_UNKNOWN, battery.faults);
	sendMavlinkMessage(message);
	_battery_status_sent++;
}

void MylinkBridge::updateTelemetry()
{
	const hrt_abstime now = hrt_absolute_time();

	if (_last_heartbeat_tx == 0 || now - _last_heartbeat_tx >= kHeartbeatInterval) {
		_last_heartbeat_tx = now;
		sendHeartbeat();
	}

	if (_last_local_position_tx == 0 || now - _last_local_position_tx >= kLocalPositionInterval) {
		_last_local_position_tx = now;
		sendLocalPositionNed();
	}

	if (_last_battery_status_tx == 0 || now - _last_battery_status_tx >= kBatteryStatusInterval) {
		_last_battery_status_tx = now;
		sendBatteryStatus();
	}
}

void MylinkBridge::readSerial()
{
	uint8_t buffer[128];

	for (unsigned drain = 0; drain < kMaximumDrainReads; ++drain) {
		errno = 0;
		const ssize_t available = _serial.bytesAvailable();

		if (available < 0) {
			_rx_errors++;
			_last_rx_errno = errno;
			return;
		}

		if (available == 0) {
			return;
		}

		const size_t requested = math::min(static_cast<size_t>(available), sizeof(buffer));
		errno = 0;
		const ssize_t bytes_read = _serial.read(buffer, requested);

		if (bytes_read < 0) {
			if (errno != EAGAIN
#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN
			    && errno != EWOULDBLOCK
#endif
			   ) {
				_rx_errors++;
				_last_rx_errno = errno;
			}

			return;
		}

		if (bytes_read == 0) {
			return;
		}

		_rx_bytes += static_cast<uint32_t>(bytes_read);

		for (ssize_t index = 0; index < bytes_read; ++index) {
			mavlink_message_t message{};
			mavlink_status_t status{};
			const uint8_t framing = mavlink_frame_char_buffer(&_rx_parser_message, &_rx_parser_status,
						buffer[index], &message, &status);

			if (framing == MAVLINK_FRAMING_OK) {
				_valid_frames++;
				handleMavlinkMessage(message);

			} else if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
				_bad_frames++;
			}
		}
	}
}

void MylinkBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_serial.close();
		exit_and_cleanup(desc);
		return;
	}

	if (!_serial.isOpen()) {
		if (!_serial.open()) {
			PX4_ERR("open %s failed; retrying", _serial.getPort());
			ScheduleDelayed(1_s);
			return;
		}

		PX4_INFO("MAVLink RX opened %s at %" PRIu32 " baud", _serial.getPort(), _serial.getBaudrate());
		ScheduleOnInterval(kRunInterval);
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
	updateGateAndCachedMessage();
	readSerial();
	const GateState state = gateState();
	const hrt_abstime now = hrt_absolute_time();
	if (!_setpoint_cache_valid && state != GateState::Closed
	    && (_v4_last_setpoint == 0 || now - _v4_last_setpoint >= kV4SetpointInterval)) {
		updateV4Setpoint(state);
	}
	forwardCommandAcks();
	updateTelemetry();
	perf_end(_loop_perf);
}

int MylinkBridge::task_spawn(int argc, char *argv[])
{
	int option_index = 1;
	int option = 0;
	const char *option_argument = nullptr;
	const char *device = nullptr;
	int baudrate = 115200;

	while ((option = px4_getopt(argc, argv, "d:b:", &option_index, &option_argument)) != EOF) {
		switch (option) {
		case 'd':
			device = option_argument;
			break;

		case 'b':
			if (px4_get_parameter_value(option_argument, baudrate) != 0) {
				PX4_ERR("invalid baudrate");
				return PX4_ERROR;
			}

			break;

		default:
			return PX4_ERROR;
		}
	}

	if (device == nullptr || !device::Serial::validatePort(device) || baudrate <= 0) {
		PX4_ERR("valid -d <device> and -b <baudrate> are required");
		return PX4_ERROR;
	}

	MylinkBridge *instance = new MylinkBridge(device, static_cast<uint32_t>(baudrate));

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int MylinkBridge::print_status()
{
	vehicle_status_s status{};
	const GateState state = gateState(&status);
	PX4_INFO("%s gate=%s arm=%u nav=%u rx=%" PRIu32 " drop=%" PRIu32 " tx=%" PRIu32 "/%" PRIu32,
		 _serial.getPort(), gateStateName(state),
		 status.arming_state == vehicle_status_s::ARMING_STATE_ARMED, status.nav_state,
		 _valid_frames, _gate_dropped_frames, _tx_frames, _tx_errors);
	PX4_INFO("offboard=%" PRIu32 "/%" PRIu32 "/%" PRIu32 " telemetry=%" PRIu32 "/%" PRIu32 "/%" PRIu32
			 "/%" PRIu32,
			 _offboard_setpoints_received, _offboard_control_mode_published, _trajectory_setpoints_published,
			 _heartbeats_sent, _command_acks_forwarded, _local_positions_sent, _battery_status_sent);
	PX4_INFO("v4 rx=%" PRIu32 " ack=%" PRIu32 " accepted=%" PRIu32 " rejected=%" PRIu32 " busy=%" PRIu32,
		 _v4_rx_command, _v4_ack_sent, _v4_accepted, _v4_rejected, _v4_busy_rejected);
	PX4_INFO("v4 actions takeoff=%" PRIu32 " move=%" PRIu32 " hold=%" PRIu32 " land=%" PRIu32,
		 _v4_takeoff_count, _v4_move_count, _v4_hold_count, _v4_land_count);
	PX4_INFO("v4 heading invalid=%" PRIu32 " abort=%" PRIu32 " reset_abort=%" PRIu32,
		 _v4_heading_invalid_reject, _v4_heading_abort, _v4_heading_reset_abort);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MylinkBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MylinkBridge::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_NAME("mylink_bridge", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mylink_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(MylinkBridge::desc, argc, argv);
}

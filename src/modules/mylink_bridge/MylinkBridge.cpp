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
#include <lib/custom_action_protocol/CustomActionProtocol.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstring>

#if defined(__PX4_POSIX)
# include <arpa/inet.h>
# include <fcntl.h>
# include <netinet/in.h>
# include <sys/socket.h>
# include <unistd.h>
#endif

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
constexpr hrt_abstime kTopDistanceTxInterval = 100_ms;
constexpr hrt_abstime kMotorOutputTxInterval = 100_ms;
constexpr hrt_abstime kMotorOutputSampleTimeout = 500_ms;
}

ModuleBase::Descriptor MylinkBridge::desc{task_spawn, custom_command, print_usage};

MylinkBridge::MylinkBridge(const char *device, uint32_t baudrate, int udp_port) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_serial(device != nullptr ? device : "/dev/null", baudrate),
	_udp_port(udp_port)
{
}

MylinkBridge::~MylinkBridge()
{
	ScheduleClear();
	#if defined(__PX4_POSIX)
	closeUdp();
	#endif
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

bool MylinkBridge::targetOk(uint8_t target_system, uint8_t target_component)
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t component_id = status.component_id > 0 ? status.component_id : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);

	return (target_system == 0 || target_system == system_id)
	       && (target_component == MAV_COMP_ID_ALL || target_component == component_id);
}

bool MylinkBridge::customTargetOk(uint8_t target_system, uint8_t target_component)
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	return (target_system == 0 || target_system == system_id)
	       && target_component == custom_action_protocol::kComponentId;
}

bool MylinkBridge::legacyControlAllowed()
{
	_custom_action_status_sub.update(&_custom_action_status);

	if (_commander_owns_control) {
		return false;
	}

	// Before the controller publishes its first status, retain the established
	// V3 behavior. Once status exists, PX4 is the authority for ownership.
	return _custom_action_status.timestamp == 0
	       || _custom_action_status.control_owner == custom_action_status_s::OWNER_LEGACY;
}

bool MylinkBridge::commandSupported(uint16_t command) const
{
	return command == MAV_CMD_NAV_LAND;
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
	ssize_t written = -1;

#if defined(__PX4_POSIX)
	if (_udp_port > 0) {
		if (_udp_fd >= 0 && _udp_peer_valid) {
			written = sendto(_udp_fd, buffer, length, 0,
					 reinterpret_cast<const sockaddr *>(&_udp_peer_addr), _udp_peer_addr_len);
		} else {
			errno = ENOTCONN;
		}

	} else
#endif
	{
		written = _serial.writeBlocking(buffer, length, 10);
	}

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
}

void MylinkBridge::sendVehicleCommandAck(const vehicle_command_ack_s &ack)
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	const uint8_t source_component = ack.command == custom_action_protocol::kMavCmdUser1
					 ? custom_action_protocol::kComponentId
					 : static_cast<uint8_t>(MAV_COMP_ID_AUTOPILOT1);

	mavlink_message_t response{};
	mavlink_msg_command_ack_pack_status(system_id, source_component, &_tx_status, &response,
					    static_cast<uint16_t>(ack.command), ack.result, ack.result_param1,
					    ack.result_param2, ack.target_system,
					    static_cast<uint8_t>(ack.target_component));
	sendMavlinkMessage(response);
}

void MylinkBridge::relayVehicleCommandAcks()
{
	vehicle_command_ack_s ack{};

	for (int i = 0; i < vehicle_command_ack_s::ORB_QUEUE_LENGTH && _vehicle_command_ack_sub.update(&ack); ++i) {
		if (!ack.from_external && ack.target_system == _remote_system
		    && ack.target_component == _remote_component
		    && ack.command <= UINT16_MAX) {
			sendVehicleCommandAck(ack);
			_relayed_command_acks++;
		}
	}
}

void MylinkBridge::relayTopDistance()
{
	// Wait until the upper computer has identified itself on this link. This
	// also prevents consuming the newest uORB sample before a SITL UDP peer is
	// available.
	if (_remote_system == 0 || _remote_component == 0) {
		return;
	}

#if defined(__PX4_POSIX)
	if (_udp_port > 0 && !_udp_peer_valid) {
		return;
	}
#endif

	const hrt_abstime now = hrt_absolute_time();

	// The flight controller continues to consume top_distance at the sensor
	// rate. Only the monitoring copy is limited to 10 Hz to keep TELEM2 light.
	if (_last_top_distance_tx != 0 && now - _last_top_distance_tx < kTopDistanceTxInterval) {
		return;
	}

	top_distance_s report{};

	if (!_top_distance_sub.update(&report) || report.timestamp == 0) {
		return;
	}

	uint64_t packed_distances = 0;
	uint8_t valid_mask = 0;

	for (uint8_t sensor_id = 0;
	     sensor_id < custom_action_protocol::kTopDistanceSensorCount;
	     ++sensor_id) {
		const uint8_t sensor_bit = 1u << sensor_id;
		const float distance_m = report.distance_m[sensor_id];

		if ((report.valid_mask & sensor_bit) == 0 || !PX4_ISFINITE(distance_m) || distance_m <= 0.f) {
			continue;
		}

		const long distance_mm = lroundf(distance_m * 1000.f);

		if (distance_mm <= 0 || distance_mm > static_cast<long>(UINT16_MAX)) {
			continue;
		}

		packed_distances |= static_cast<uint64_t>(distance_mm)
				    << (sensor_id * custom_action_protocol::kTopDistanceArrayDistanceBits);
		valid_mask |= sensor_bit;
	}

	const uint32_t metadata = custom_action_protocol::kTopDistanceArrayMarker
				  | custom_action_protocol::kTopDistanceArrayVersion
				  | (static_cast<uint32_t>(valid_mask)
				     << custom_action_protocol::kTopDistanceArrayValidShift)
				  | (static_cast<uint32_t>(report.sequence)
				     & custom_action_protocol::kTopDistanceArraySequenceMask);

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;
	mavlink_message_t message{};
	mavlink_msg_ping_pack_status(system_id, custom_action_protocol::kComponentId,
				     &_tx_status, &message, packed_distances, metadata,
				     _remote_system, static_cast<uint8_t>(_remote_component));
	sendMavlinkMessage(message);
	_last_top_distance_tx = now;
	_relayed_top_distance_frames++;
}

void MylinkBridge::relayMotorOutputs()
{
	if (_remote_system == 0 || _remote_component == 0) {
		return;
	}

#if defined(__PX4_POSIX)
	if (_udp_port > 0 && !_udp_peer_valid) {
		return;
	}
#endif

	const hrt_abstime now = hrt_absolute_time();

	if (_last_motor_output_tx != 0 && now - _last_motor_output_tx < kMotorOutputTxInterval) {
		return;
	}

	vehicle_status_s vehicle_status{};
	const bool status_available = _vehicle_status_sub.copy(&vehicle_status) && vehicle_status.timestamp != 0;
	const bool armed = status_available
			   && vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	actuator_motors_s motors{};
	const bool motors_available = _actuator_motors_sub.copy(&motors) && motors.timestamp != 0;
	const bool motors_fresh = motors_available && now >= motors.timestamp
				  && now - motors.timestamp <= kMotorOutputSampleTimeout;
	uint64_t packed_outputs = 0;
	uint8_t valid_mask = 0;

	for (uint8_t motor_index = 0;
	     motor_index < custom_action_protocol::kMotorOutputCount;
	     ++motor_index) {
		float normalized = 0.f;
		bool valid = status_available && !armed;

		if (armed && motors_fresh && PX4_ISFINITE(motors.control[motor_index])) {
			normalized = math::constrain(motors.control[motor_index], 0.f, 1.f);
			valid = true;
		}

		if (valid) {
			// The dedicated MyLink transport is used by SITL, where there are no
			// physical PWM pins. Report a conventional PWM-equivalent command.
			const uint16_t pwm_us = static_cast<uint16_t>(lroundf(
						custom_action_protocol::kMotorPwmSimMinimumUs
						+ custom_action_protocol::kMotorPwmSimRangeUs * normalized));
			packed_outputs |= static_cast<uint64_t>(pwm_us)
					  << (motor_index * custom_action_protocol::kMotorOutputArrayValueBits);
			valid_mask |= 1u << motor_index;
		}
	}

	_motor_output_sequence++;
	const uint32_t metadata = custom_action_protocol::kMotorOutputArrayMarker
				  | custom_action_protocol::kMotorOutputArrayVersion
				  | (static_cast<uint32_t>(valid_mask)
				     << custom_action_protocol::kMotorOutputArrayValidShift)
				  | (armed ? custom_action_protocol::kMotorOutputArrayArmedFlag : 0u)
				  | (static_cast<uint32_t>(_motor_output_sequence)
				     & custom_action_protocol::kMotorOutputArraySequenceMask);

	const uint8_t system_id = vehicle_status.system_id > 0 ? vehicle_status.system_id : 1;
	mavlink_message_t message{};
	mavlink_msg_ping_pack_status(system_id, custom_action_protocol::kComponentId,
				     &_tx_status, &message, packed_outputs, metadata,
				     _remote_system, static_cast<uint8_t>(_remote_component));
	sendMavlinkMessage(message);
	_last_motor_output_tx = now;
	_relayed_motor_output_frames++;
}

void MylinkBridge::relayCustomActionStatus()
{
	if (_remote_system == 0 || _remote_component == 0) {
		return;
	}

#if defined(__PX4_POSIX)
	if (_udp_port > 0 && !_udp_peer_valid) {
		return;
	}
#endif

	custom_action_status_s status{};

	if (!_custom_action_status_sub.copy(&status) || status.timestamp == 0
	    || status.timestamp == _last_custom_status_timestamp) {
		return;
	}

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);
	const uint8_t system_id = vehicle_status.system_id > 0 ? vehicle_status.system_id : 1;
	mavlink_message_t state_message{};
	const uint32_t state_metadata = custom_action_protocol::kCustomStatusMarker
					| custom_action_protocol::kCustomStatusVersion
					| ((static_cast<uint32_t>(status.state)
					    << custom_action_protocol::kCustomStatusStateShift)
					   & custom_action_protocol::kCustomStatusStateMask)
					| ((static_cast<uint32_t>(status.control_owner)
					    << custom_action_protocol::kCustomStatusOwnerShift)
					   & custom_action_protocol::kCustomStatusOwnerMask)
					| (status.active ? custom_action_protocol::kCustomStatusActiveFlag : 0u)
					| ((static_cast<uint32_t>(status.reason)
					    << custom_action_protocol::kCustomStatusReasonShift)
					   & custom_action_protocol::kCustomStatusReasonMask)
					| (static_cast<uint32_t>(++_custom_status_sequence)
					   & custom_action_protocol::kCustomStatusSequenceMask);
	mavlink_msg_ping_pack_status(system_id, custom_action_protocol::kComponentId,
				     &_tx_status, &state_message, status.handover_id, state_metadata,
				     _remote_system, static_cast<uint8_t>(_remote_component));
	sendMavlinkMessage(state_message);

	const uint16_t target_gain = static_cast<uint16_t>(math::constrain(
				     status.pressure_target_gain * custom_action_protocol::kPressureDetailGainScale,
				     0.f, static_cast<float>(UINT16_MAX)) + 0.5f);
	const uint16_t applied_gain = static_cast<uint16_t>(math::constrain(
				      status.pressure_applied_gain * custom_action_protocol::kPressureDetailGainScale,
				      0.f, static_cast<float>(UINT16_MAX)) + 0.5f);
	const uint16_t progress = static_cast<uint16_t>(math::constrain(
				  status.pressure_progress * custom_action_protocol::kPressureDetailProgressScale,
				  0.f, custom_action_protocol::kPressureDetailProgressScale) + 0.5f);
	const uint16_t pressure_time_ms = static_cast<uint16_t>(math::constrain(
					 status.pressure_time_s * custom_action_protocol::kPressureDetailTimeScale,
					 0.f, static_cast<float>(UINT16_MAX)) + 0.5f);
	const uint64_t packed_pressure = static_cast<uint64_t>(target_gain)
					 | (static_cast<uint64_t>(applied_gain) << custom_action_protocol::kPressureDetailValueBits)
					 | (static_cast<uint64_t>(progress) << (2 * custom_action_protocol::kPressureDetailValueBits))
					 | (static_cast<uint64_t>(pressure_time_ms) << (3 * custom_action_protocol::kPressureDetailValueBits));
	const uint32_t pressure_metadata = custom_action_protocol::kPressureDetailMarker
					   | custom_action_protocol::kPressureDetailVersion
					   | ((static_cast<uint32_t>(status.trim_source)
					       << custom_action_protocol::kPressureDetailTrimSourceShift)
					      & custom_action_protocol::kPressureDetailTrimSourceMask)
					   | ((static_cast<uint32_t>(status.trim_candidate_mask)
					       << custom_action_protocol::kPressureDetailCandidateShift)
					      & custom_action_protocol::kPressureDetailCandidateMask)
					   | ((static_cast<uint32_t>(status.limiting_motor)
					       << custom_action_protocol::kPressureDetailLimitingMotorShift)
					      & custom_action_protocol::kPressureDetailLimitingMotorMask)
					   | (status.pressure_limited ? custom_action_protocol::kPressureDetailLimitedFlag : 0u)
					   | (status.pressure_ramp_complete ? custom_action_protocol::kPressureDetailCompleteFlag : 0u)
					   | custom_action_protocol::kPressureDetailValidFlag
					   | (static_cast<uint32_t>(++_pressure_detail_sequence)
					      & custom_action_protocol::kPressureDetailSequenceMask);
	mavlink_message_t pressure_message{};
	mavlink_msg_ping_pack_status(system_id, custom_action_protocol::kComponentId,
				     &_tx_status, &pressure_message, packed_pressure, pressure_metadata,
				     _remote_system, static_cast<uint8_t>(_remote_component));
	sendMavlinkMessage(pressure_message);

	_last_custom_status_timestamp = status.timestamp;
	_relayed_custom_status_frames += 2;
}

void MylinkBridge::handlePing(const mavlink_message_t &message)
{
	mavlink_ping_t ping{};
	mavlink_msg_ping_decode(&message, &ping);
	_handled_messages++;

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
}

void MylinkBridge::handleHeartbeat(const mavlink_message_t &message)
{
	(void)message;
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
}

void MylinkBridge::handleCommandLong(const mavlink_message_t &message)
{
	mavlink_command_long_t command{};
	mavlink_msg_command_long_decode(&message, &command);

	if (command.command == custom_action_protocol::kMavCmdUser1) {
		handleCustomActionCommand(message, command);
		return;
	}

	if (!targetOk(command.target_system, command.target_component)) {
		return;
	}

	if (command.command == MAV_CMD_DO_SET_MODE) {
		handleOffboardModeCommand(message, command);
		return;
	}

	if (command.command == MAV_CMD_NAV_LAND) {
		_commander_owns_control = true;
		publishVehicleCommand(message, command, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
		_handled_messages++;
		return;
	}

	if (command.command == MAV_CMD_DO_MOTOR_TEST || command.command == MAV_CMD_DO_SET_ACTUATOR) {
		sendCommandAck(command.command, MAV_RESULT_DENIED, UINT8_MAX, 0, message.sysid, message.compid);
		_unsupported_messages++;
		return;
	}

	if (!commandSupported(command.command)) {
		sendCommandAck(command.command, MAV_RESULT_UNSUPPORTED, UINT8_MAX, 0, message.sysid, message.compid);
		_unsupported_messages++;
		return;
	}
}

void MylinkBridge::handleCustomActionCommand(const mavlink_message_t &message,
		const mavlink_command_long_t &command)
{
	if (!customTargetOk(command.target_system, command.target_component)) {
		return;
	}

	publishVehicleCommand(message, command, custom_action_protocol::kMavCmdUser1);
	_custom_action_commands++;
	_handled_messages++;
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

	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.param1 = command.param1;
	vehicle_command.param2 = command.param2;
	vehicle_command.param3 = command.param3;
	vehicle_command.param4 = command.param4;
	vehicle_command.param5 = command.param5;
	vehicle_command.param6 = command.param6;
	vehicle_command.param7 = command.param7;
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vehicle_command.target_system = command.target_system;
	vehicle_command.target_component = command.target_component;
	vehicle_command.source_system = message.sysid;
	vehicle_command.source_component = message.compid;
	vehicle_command.confirmation = command.confirmation;
	vehicle_command.from_external = true;
	_vehicle_command_pub.publish(vehicle_command);
	_offboard_mode_requests++;
	_handled_messages++;
}

void MylinkBridge::handleCommandInt(const mavlink_message_t &message)
{
	mavlink_command_int_t command{};
	mavlink_msg_command_int_decode(&message, &command);

	if (targetOk(command.target_system, command.target_component)) {
		sendCommandAck(command.command, MAV_RESULT_COMMAND_LONG_ONLY, UINT8_MAX, 0,
			       message.sysid, message.compid);
		_unsupported_messages++;
	}
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

void MylinkBridge::handleSetPositionTargetLocalNed(const mavlink_message_t &message, GateState state)
{
	if (!legacyControlAllowed()) {
		_legacy_setpoints_blocked++;
		return;
	}

	// Direct motor control owns the Offboard control type while it is active.
	// A parallel velocity stream must not switch control allocation back on.
	if (_direct_motor_active || _direct_motor_stopping) {
		return;
	}

	trajectory_setpoint_s setpoint{};
	offboard_control_mode_s control_mode{};

	if (!decodeLocalNedSetpoint(message, setpoint, control_mode)) {
		_invalid_setpoints++;
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
	// V3 never queues or replays control input. Local-NED messages are handled
	// only when received, so stopping the sender stops the heartbeat/setpoint.
	return false;
}

void MylinkBridge::cacheMessage(const mavlink_message_t &message)
{
	_cached_message = message;
	_cache_valid = true;
	_cached_updates++;
}

void MylinkBridge::clearSessionState()
{
	releaseDirectMotorControl(false);
	_cached_message = {};
	_latest_command = {};
	_latest_setpoint = {};
	_cache_valid = false;
	_setpoint_cache_valid = false;
	_last_setpoint_rx = 0;
	_event_flags = EventNone;
	_latest_event_command = 0;
	_commander_owns_control = false;
	_session_resets++;
}

void MylinkBridge::processMavlinkMessage(const mavlink_message_t &message)
{
	switch (message.msgid) {
	case MAVLINK_MSG_ID_PING:
		handlePing(message);
		break;

	case MAVLINK_MSG_ID_COMMAND_LONG:
		handleCommandLong(message);
		break;

	case MAVLINK_MSG_ID_COMMAND_INT:
		handleCommandInt(message);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		handleSetPositionTargetLocalNed(message, GateState::Active);
		break;

	default:
		_unsupported_messages++;
		break;
	}
}

void MylinkBridge::handleMavlinkMessage(const mavlink_message_t &message)
{
	_remote_system = message.sysid;
	_remote_component = message.compid;

	if (message.msgid == MAVLINK_MSG_ID_PING) {
		handlePing(message);
		return;
	}

	if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		handleHeartbeat(message);
		return;
	}

	// CUSTOM1 must receive an explicit rejection even when disarmed. The custom
	// controller, not this bridge gate, owns its flight-state validation.
	if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
		mavlink_command_long_t command{};
		mavlink_msg_command_long_decode(&message, &command);

		if (command.command == custom_action_protocol::kMavCmdUser1) {
			handleCommandLong(message);
			return;
		}
	}

	const GateState state = gateState();

	if (state == GateState::Closed) {
		_gate_dropped_frames++;
		return;
	}

	if (state == GateState::CacheOnly) {
		if (message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
			handleSetPositionTargetLocalNed(message, state);

		} else if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
			// Mode requests and LAND are forwarded immediately. All other
			// commands are rejected; V3 has no command queue.
			handleCommandLong(message);

		} else if (cacheableMessage(message)) {
			cacheMessage(message);

		} else {
			_unsupported_messages++;
		}

		return;
	}

	processMavlinkMessage(message);
}

void MylinkBridge::updateGateAndCachedMessage()
{
	vehicle_status_s status{};
	const GateState state = gateState(&status);
	const bool armed = status.timestamp != 0
			   && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (_was_armed && !armed) {
		clearSessionState();
	}

	_was_armed = armed;
	_custom_action_status_sub.update(&_custom_action_status);


	updateDirectMotorControl(state);

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

#if defined(__PX4_POSIX)
bool MylinkBridge::openUdp()
{
	_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_udp_fd < 0) {
		_last_rx_errno = errno;
		return false;
	}

	const int reuse = 1;
	(void)setsockopt(_udp_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
	const int flags = fcntl(_udp_fd, F_GETFL, 0);

	if (flags < 0 || fcntl(_udp_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
		_last_rx_errno = errno;
		closeUdp();
		return false;
	}

	sockaddr_in address{};
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl(INADDR_ANY);
	address.sin_port = htons(static_cast<uint16_t>(_udp_port));

	if (bind(_udp_fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
		_last_rx_errno = errno;
		closeUdp();
		return false;
	}

	_udp_peer_valid = false;
	return true;
}

void MylinkBridge::closeUdp()
{
	if (_udp_fd >= 0) {
		::close(_udp_fd);
		_udp_fd = -1;
	}

	_udp_peer_valid = false;
	_udp_peer_addr_len = 0;
}

void MylinkBridge::readUdp()
{
	uint8_t buffer[2048];

	for (unsigned drain = 0; drain < kMaximumDrainReads; ++drain) {
		sockaddr_storage peer{};
		socklen_t peer_length = sizeof(peer);
		errno = 0;
		const ssize_t bytes_read = recvfrom(_udp_fd, buffer, sizeof(buffer), 0,
					      reinterpret_cast<sockaddr *>(&peer), &peer_length);

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

		_udp_peer_addr = peer;
		_udp_peer_addr_len = peer_length;
		_udp_peer_valid = true;
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
#endif

void MylinkBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		#if defined(__PX4_POSIX)
		closeUdp();
		#endif
		_serial.close();
		exit_and_cleanup(desc);
		return;
	}

	#if defined(__PX4_POSIX)
	if (_udp_port > 0) {
		if (_udp_fd < 0) {
			if (!openUdp()) {
				PX4_ERR("open UDP port %d failed (%d); retrying", _udp_port, _last_rx_errno);
				ScheduleDelayed(1_s);
				return;
			}

			PX4_INFO("MAVLink UDP RX listening on 0.0.0.0:%d", _udp_port);
			ScheduleOnInterval(kRunInterval);
		}

		perf_begin(_loop_perf);
		perf_count(_loop_interval_perf);
		updateGateAndCachedMessage();
		relayVehicleCommandAcks();
		relayTopDistance();
		relayMotorOutputs();
		relayCustomActionStatus();
		readUdp();
		perf_end(_loop_perf);
		return;
	}
	#endif

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
	relayVehicleCommandAcks();
	relayTopDistance();
	relayMotorOutputs();
	relayCustomActionStatus();
	readSerial();
	perf_end(_loop_perf);
}

int MylinkBridge::task_spawn(int argc, char *argv[])
{
	int option_index = 1;
	int option = 0;
	const char *option_argument = nullptr;
	const char *device = nullptr;
	int baudrate = 115200;
	int udp_port = -1;

	while ((option = px4_getopt(argc, argv, "d:b:u:", &option_index, &option_argument)) != EOF) {
		switch (option) {
		case 'd':
			device = option_argument;
			break;

		case 'u':
			if (px4_get_parameter_value(option_argument, udp_port) != 0) {
				PX4_ERR("invalid UDP port");
				return PX4_ERROR;
			}

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

	const bool serial_valid = device != nullptr && device::Serial::validatePort(device) && baudrate > 0;
	const bool udp_valid = udp_port > 0 && udp_port <= UINT16_MAX;

#if !defined(__PX4_POSIX)
	if (udp_valid) {
		PX4_ERR("UDP transport is only available in SITL/POSIX");
		return PX4_ERROR;
	}
#endif

	if (serial_valid == udp_valid) {
		PX4_ERR("select exactly one transport: -d <device> or -u <UDP port>");
		return PX4_ERROR;
	}

	MylinkBridge *instance = new MylinkBridge(device, static_cast<uint32_t>(baudrate), udp_port);

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
	const char *state_name = state == GateState::Active ? "ACTIVE" :
				state == GateState::CacheOnly ? "CACHE_ONLY" : "CLOSED";
	#if defined(__PX4_POSIX)
	if (_udp_port > 0) {
		PX4_INFO("transport=UDP port=%d peer=%s MAVLink2 gate=%s armed=%s nav_state=%u",
			 _udp_port, _udp_peer_valid ? "connected" : "waiting", state_name,
			 status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? "yes" : "no",
			 status.nav_state);
	} else
	#endif
	{
		PX4_INFO("transport=UART port=%s baud=%" PRIu32 " MAVLink2 gate=%s armed=%s nav_state=%u",
		 _serial.getPort(), _serial.getBaudrate(), state_name,
		 status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? "yes" : "no",
		 status.nav_state);
	}
	PX4_INFO("rx: bytes=%" PRIu32 " valid_frames=%" PRIu32 " bad_frames=%" PRIu32
		 " gate_dropped=%" PRIu32 " cached_updates=%" PRIu32 " handled=%" PRIu32 " unsupported=%" PRIu32,
		 _rx_bytes, _valid_frames, _bad_frames, _gate_dropped_frames,
		 _cached_updates, _handled_messages, _unsupported_messages);
	PX4_INFO("session: cache_valid=%s cached_msgid=%" PRIu32 " event_flags=0x%08" PRIx32
		 " latest_command=%u setpoint_cache=%s resets=%" PRIu32,
		 _cache_valid ? "yes" : "no", _cache_valid ? _cached_message.msgid : 0,
		 _event_flags, _latest_event_command, _setpoint_cache_valid ? "yes" : "no", _session_resets);
	const hrt_abstime now = hrt_absolute_time();
	const uint64_t setpoint_age_ms = _last_setpoint_rx > 0 && now >= _last_setpoint_rx
					 ? (now - _last_setpoint_rx) / 1000 : 0;
	PX4_INFO("offboard: received=%" PRIu32 " ocm_published=%" PRIu32
		 " trajectory_published=%" PRIu32 " mode_requests=%" PRIu32
		 " invalid=%" PRIu32 " last_age_ms=%" PRIu64,
		 _offboard_setpoints_received, _offboard_control_mode_published,
		 _trajectory_setpoints_published, _offboard_mode_requests,
		 _invalid_setpoints, setpoint_age_ms);
	PX4_INFO("custom: commands=%" PRIu32 " owner=%u state=%u handover=%u legacy_blocked=%" PRIu32
		 " ack_relayed=%" PRIu32,
		 _custom_action_commands, _custom_action_status.control_owner, _custom_action_status.state,
		 _custom_action_status.handover_id, _legacy_setpoints_blocked, _relayed_command_acks);
	PX4_INFO("top_distance: relayed=%" PRIu32 " rate_limit=10Hz last_tx_age_ms=%" PRIu64,
		 _relayed_top_distance_frames,
		 _last_top_distance_tx > 0 && hrt_absolute_time() >= _last_top_distance_tx
		 ? (hrt_absolute_time() - _last_top_distance_tx) / 1000 : 0);
	PX4_INFO("motor_output: relayed=%" PRIu32 " rate_limit=10Hz last_tx_age_ms=%" PRIu64,
		 _relayed_motor_output_frames,
		 _last_motor_output_tx > 0 && hrt_absolute_time() >= _last_motor_output_tx
		 ? (hrt_absolute_time() - _last_motor_output_tx) / 1000 : 0);
	PX4_INFO("custom_status: relayed_frames=%" PRIu32 " rate_limit=5Hz last_tx_age_ms=%" PRIu64,
		 _relayed_custom_status_frames,
		 _last_custom_status_timestamp > 0 && hrt_absolute_time() >= _last_custom_status_timestamp
		 ? (hrt_absolute_time() - _last_custom_status_timestamp) / 1000 : 0);
	PX4_INFO("events: takeoff=%u land=%u speed=%u pause=%u continue=%u rtl=%u mission_start=%u",
		 (_event_flags & EventTakeoff) != 0, (_event_flags & EventLand) != 0,
		 (_event_flags & EventSpeed) != 0, (_event_flags & EventPause) != 0,
		 (_event_flags & EventContinue) != 0, (_event_flags & EventRtl) != 0,
		 (_event_flags & EventMissionStart) != 0);
	PX4_INFO("motor_test: event=%u active=%s function=%u throttle=%.1f%% commands=%" PRIu32
		 " invalid=%" PRIu32 " setpoints=%" PRIu32 " releases=%" PRIu32,
		 (_event_flags & EventMotorThrottle) != 0, _direct_motor_active ? "yes" : "no",
		 _direct_motor_active ? kDirectMotorCount : 0, (double)_motor_throttle_percent, _motor_test_commands,
		 _invalid_motor_test_commands, _direct_motor_setpoints_published, _motor_test_releases);
	PX4_INFO("tx: frames=%" PRIu32 " bytes=%" PRIu32 " errors=%" PRIu32
		 " rx_errors=%" PRIu32 " last_rx_errno=%d last_tx_errno=%d",
		 _tx_frames, _tx_bytes, _tx_errors, _rx_errors, _last_rx_errno, _last_tx_errno);
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
	PRINT_MODULE_USAGE_PARAM_INT('u', 14541, 1, 65535, "SITL UDP listen port", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mylink_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(MylinkBridge::desc, argc, argv);
}

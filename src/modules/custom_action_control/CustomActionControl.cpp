/****************************************************************************
 * SEARCH_TOP controller implementation.
 ****************************************************************************/

#include "CustomActionControl.hpp"

#include <geo/geo.h>
#include <lib/custom_action_protocol/CustomActionProtocol.hpp>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/log.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

using namespace time_literals;
using custom_action_protocol::Command;
using custom_action_protocol::Result;

namespace
{
constexpr hrt_abstime kRunInterval = 20_ms;
constexpr hrt_abstime kLocalPositionTimeout = 500_ms;
constexpr hrt_abstime kStatusInterval = 500_ms;
constexpr hrt_abstime kLandCommandRetryInterval = 500_ms;
constexpr uint8_t kContactThresholdFramesRequired = 3;
constexpr uint8_t kContactLostFramesRequired = 3;
const char *reasonName(uint8_t reason)
{
	switch (reason) {
	case custom_action_status_s::REASON_DIRECTION: return "direction";
	case custom_action_status_s::REASON_MAX_DISTANCE: return "max distance";
	case custom_action_status_s::REASON_TIMEOUT: return "timeout";
	case custom_action_status_s::REASON_SENSOR_TIMEOUT: return "sensor timeout";
	case custom_action_status_s::REASON_ESTIMATOR: return "estimator invalid";
	case custom_action_status_s::REASON_HEADING_RESET: return "heading reset";
	case custom_action_status_s::REASON_LAND: return "land";
	case custom_action_status_s::REASON_TOP_DISTANCE: return "top distance";
	case custom_action_status_s::REASON_CONTACT_LOST: return "contact lost";
	default: return "none";
	}
}
}

ModuleBase::Descriptor CustomActionControl::desc{task_spawn, custom_command, print_usage};

CustomActionControl::CustomActionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

CustomActionControl::~CustomActionControl()
{
	ScheduleClear();
}

bool CustomActionControl::init()
{
	publishStatus(true);
	ScheduleOnInterval(kRunInterval);
	return true;
}

bool CustomActionControl::localStateValid() const
{
	const hrt_abstime now = hrt_absolute_time();
	return _local_position.timestamp != 0 && now >= _local_position.timestamp
	       && now - _local_position.timestamp <= kLocalPositionTimeout
	       && _local_position.xy_valid && _local_position.z_valid
	       && PX4_ISFINITE(_local_position.x) && PX4_ISFINITE(_local_position.y)
	       && PX4_ISFINITE(_local_position.z) && PX4_ISFINITE(_local_position.heading);
}

bool CustomActionControl::flightStateAllowsCustom() const
{
	return _param_enabled.get() && _vehicle_status.timestamp != 0
	       && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	       && localStateValid();
}

bool CustomActionControl::sensorFresh(hrt_abstime now) const
{
	const hrt_abstime timeout = static_cast<hrt_abstime>(_param_sensor_timeout.get() * 1_s);

	if (_top_distance.valid_mask != 0x0F || _top_distance.timestamp == 0
	    || now < _top_distance.timestamp || now - _top_distance.timestamp > timeout) {
		return false;
	}

	for (uint8_t sensor = 0; sensor < 4; ++sensor) {
		if (_top_distance.timestamp_sample[sensor] == 0 || now < _top_distance.timestamp_sample[sensor]
		    || now - _top_distance.timestamp_sample[sensor] > timeout
		    || !PX4_ISFINITE(_top_distance.distance_m[sensor])) {
			return false;
		}
	}

	return true;
}

float CustomActionControl::minimumTopDistance() const
{
	float minimum_distance = INFINITY;

	for (float distance : _top_distance.distance_m) {
		minimum_distance = math::min(minimum_distance, distance);
	}

	return minimum_distance;
}

float CustomActionControl::contactReferenceDistance() const
{
	float sorted[4] {
		_top_distance.distance_m[0], _top_distance.distance_m[1],
		_top_distance.distance_m[2], _top_distance.distance_m[3]
	};

	for (int i = 1; i < 4; ++i) {
		const float value = sorted[i];
		int j = i - 1;

		while (j >= 0 && sorted[j] > value) {
			sorted[j + 1] = sorted[j];
			--j;
		}

		sorted[j + 1] = value;
	}

	return 0.5f * (sorted[1] + sorted[2]);
}

uint8_t CustomActionControl::closeSensorCount(float threshold) const
{
	uint8_t count = 0;

	for (float distance : _top_distance.distance_m) {
		if (PX4_ISFINITE(distance) && distance <= threshold) {
			++count;
		}
	}

	return count;
}

uint8_t CustomActionControl::releasedSensorCount(float distance_increase) const
{
	uint8_t count = 0;

	for (uint8_t sensor = 0; sensor < 4; ++sensor) {
		if (PX4_ISFINITE(_detach_start_top_distance[sensor])
		    && PX4_ISFINITE(_top_distance.distance_m[sensor])
		    && _top_distance.distance_m[sensor] - _detach_start_top_distance[sensor] >= distance_increase) {
			++count;
		}
	}

	return count;
}

void CustomActionControl::updateFilteredTopDistance()
{
	const float measured_distance = minimumTopDistance();
	const float contact_distance = contactReferenceDistance();

	if (!PX4_ISFINITE(measured_distance) || !PX4_ISFINITE(contact_distance)) {
		return;
	}

	if (!PX4_ISFINITE(_filtered_top_distance)) {
		_filtered_top_distance = measured_distance;

	} else {
		_filtered_top_distance += _param_top_filter.get() * (measured_distance - _filtered_top_distance);
	}

	if (!PX4_ISFINITE(_filtered_contact_distance)) {
		_filtered_contact_distance = contact_distance;

	} else {
		_filtered_contact_distance += _param_top_filter.get() * (contact_distance - _filtered_contact_distance);
	}
}

bool CustomActionControl::isPrecontactState() const
{
	return _state == custom_action_status_s::STATE_SEARCH_TOP
	       || _state == custom_action_status_s::STATE_TOP_APPROACH
	       || _state == custom_action_status_s::STATE_CONTACT_VERIFY;
}

bool CustomActionControl::isDetachState() const
{
	return _state == custom_action_status_s::STATE_PRESS_RELEASE
	       || _state == custom_action_status_s::STATE_CLEARANCE_DESCEND
	       || _state == custom_action_status_s::STATE_ATTITUDE_RECOVER;
}

bool CustomActionControl::attitudeBalanced(hrt_abstime now) const
{
	if (_vehicle_attitude.timestamp == 0 || now < _vehicle_attitude.timestamp
	    || now - _vehicle_attitude.timestamp > kLocalPositionTimeout
	    || _vehicle_angular_velocity.timestamp == 0 || now < _vehicle_angular_velocity.timestamp
	    || now - _vehicle_angular_velocity.timestamp > kLocalPositionTimeout) {
		return false;
	}

	const matrix::Eulerf euler{matrix::Quatf{_vehicle_attitude.q}};
	const float angle_limit = math::radians(_param_balance_angle.get());
	const float rate_limit = math::radians(_param_balance_rate.get());

	return PX4_ISFINITE(euler.phi()) && PX4_ISFINITE(euler.theta())
	       && fabsf(euler.phi()) <= angle_limit && fabsf(euler.theta()) <= angle_limit
	       && PX4_ISFINITE(_vehicle_angular_velocity.xyz[0])
	       && PX4_ISFINITE(_vehicle_angular_velocity.xyz[1])
	       && fabsf(_vehicle_angular_velocity.xyz[0]) <= rate_limit
	       && fabsf(_vehicle_angular_velocity.xyz[1]) <= rate_limit;
}

float CustomActionControl::activeClimbVelocity() const
{
	if (_state == custom_action_status_s::STATE_TOP_APPROACH) {
		return _param_approach_velocity.get();
	}

	if (_state == custom_action_status_s::STATE_CONTACT_VERIFY) {
		return _param_verify_velocity.get();
	}

	return _param_top_velocity.get();
}

float CustomActionControl::pressureOffset(hrt_abstime now) const
{
	if (_press_started == 0 || now < _press_started) {
		return 0.f;
	}

	const float elapsed = static_cast<float>(now - _press_started) * 1e-6f;
	return math::min(_param_press_add.get(), _param_press_ramp.get() * elapsed);
}

void CustomActionControl::requestLand(hrt_abstime now, bool forced)
{
	if (_land_command_last_sent != 0 && now >= _land_command_last_sent
	    && now - _land_command_last_sent < kLandCommandRetryInterval) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
	command.target_system = _vehicle_status.system_id;
	command.target_component = _vehicle_status.component_id;
	command.source_system = _vehicle_status.system_id;
	command.source_component = _vehicle_status.component_id;
	command.confirmation = _land_command_sent;
	command.from_external = false;
	_land_command_sent = true;
	_land_command_last_sent = now;
	_vehicle_command_pub.publish(command);

	if (!_land_complete_notified) {
		publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
				static_cast<uint8_t>(Result::DetachComplete), _active_request_id,
				_active_source_system, _active_source_component);
		_land_complete_notified = true;
	}

	PX4_WARN("[CLEARANCE_DESCEND] request LAND%s", forced ? " (forced clearance)" : "");
}

bool CustomActionControl::captureHoldPoint()
{
	if (!localStateValid()) {
		return false;
	}

	_hold_x = _local_position.x;
	_hold_y = _local_position.y;
	_hold_z = _local_position.z;
	_locked_yaw = _local_position.heading;
	return true;
}

void CustomActionControl::publishAck(const vehicle_command_s &command, uint8_t result, uint8_t project_result)
{
	publishAsyncAck(result, project_result, static_cast<uint16_t>(lroundf(command.param3)),
			command.source_system, command.source_component);
}

void CustomActionControl::publishAsyncAck(uint8_t result, uint8_t project_result, uint16_t request_id,
		uint8_t target_system, uint16_t target_component)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = custom_action_protocol::kMavCmdUser1;
	ack.result = result;
	ack.result_param1 = 0;
	ack.result_param2 = custom_action_protocol::encodeResult(request_id, static_cast<Result>(project_result));
	ack.target_system = target_system;
	ack.target_component = target_component;
	ack.from_external = false;
	_vehicle_command_ack_pub.publish(ack);
}

void CustomActionControl::startSearchTop(const vehicle_command_s &command, uint16_t request_id)
{
	const hrt_abstime now = hrt_absolute_time();

	if (_owner != custom_action_status_s::OWNER_LEGACY || _state != custom_action_status_s::STATE_INACTIVE
	    || !flightStateAllowsCustom() || !sensorFresh(now) || !captureHoldPoint()) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::None));
		return;
	}

	_state = custom_action_status_s::STATE_SEARCH_TOP;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_reason = custom_action_status_s::REASON_NONE;
	_handover_id = 0;
	_active_request_id = request_id;
	_active_source_system = command.source_system;
	_active_source_component = command.source_component;
	_start_z = _hold_z;
	_heading_reset_counter = _local_position.heading_reset_counter;
	_search_started = now;
	_last_top_distance_sequence = _top_distance.sequence;
	_filtered_top_distance = minimumTopDistance();
	_filtered_contact_distance = contactReferenceDistance();
	_verify_min_distance = NAN;
	_verify_max_distance = NAN;
	_release_start_offset = 0.f;
	_release_hold_offset = 0.f;
	_detach_start_z = _hold_z;

	for (float &distance : _detach_start_top_distance) {
		distance = NAN;
	}

	_contact_threshold_frames = 0;
	_contact_lost_frames = 0;
	_detach_reason = custom_action_status_s::REASON_NONE;
	_verify_started = 0;
	_press_started = 0;
	_release_started = 0;
	_clearance_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	_land_command_last_sent = 0;
	_land_command_sent = false;
	_land_complete_notified = false;
	_handover_started = 0;
	PX4_INFO("[SEARCH_TOP] entered xyz=(%.2f, %.2f, %.2f) yaw=%.2f",
		 (double)_hold_x, (double)_hold_y, (double)_hold_z, (double)_locked_yaw);
	publishStatus(true);
	publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
		   static_cast<uint8_t>(Result::CustomStarted));
}

void CustomActionControl::beginHandover(uint8_t reason, uint16_t handover_id,
		uint8_t target_system, uint16_t target_component, bool notify_pending)
{
	if (!captureHoldPoint()) {
		releaseToLegacy(reason);
		return;
	}

	_state = custom_action_status_s::STATE_INACTIVE;
	_owner = custom_action_status_s::OWNER_HANDOVER;
	_reason = reason;
	_handover_id = handover_id == 0 ? 1 : handover_id;
	_handover_started = hrt_absolute_time();
	_verify_started = 0;
	_press_started = 0;
	_release_start_offset = 0.f;
	_release_hold_offset = 0.f;
	_release_started = 0;
	_clearance_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	_land_command_last_sent = 0;
	_land_command_sent = false;
	_land_complete_notified = false;
	_contact_threshold_frames = 0;
	_contact_lost_frames = 0;
	PX4_WARN("[CUSTOM] handover id=%u reason=%s(%u) hold=(%.2f, %.2f, %.2f)",
		 _handover_id, reasonName(reason), reason,
		 (double)_hold_x, (double)_hold_y, (double)_hold_z);
	publishStatus(true);

	if (notify_pending) {
		publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
				static_cast<uint8_t>(Result::HandoverPending), _handover_id,
				target_system, target_component);
	}
}

void CustomActionControl::enterPressRelease(hrt_abstime now)
{
	_state = custom_action_status_s::STATE_PRESS_RELEASE;
	_release_started = now;
	_clearance_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	PX4_INFO("[PRESS_RELEASE] offset=%.3f hold=%.3f ramp=%.3f/s",
		 (double)_release_start_offset, (double)_release_hold_offset,
		 (double)_param_release_ramp.get());
	publishStatus(true);
}

void CustomActionControl::enterClearanceDescend(hrt_abstime now)
{
	_state = custom_action_status_s::STATE_CLEARANCE_DESCEND;
	_release_start_offset = 0.f;
	_release_hold_offset = 0.f;
	_release_started = 0;
	_detach_start_z = _local_position.z;

	for (uint8_t sensor = 0; sensor < 4; ++sensor) {
		_detach_start_top_distance[sensor] = sensorFresh(now) ? _top_distance.distance_m[sensor] : NAN;
	}

	_clearance_started = now;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	_land_command_last_sent = 0;
	_land_command_sent = false;
	_land_complete_notified = false;
	PX4_INFO("[CLEARANCE_DESCEND] downward speed=%.2f m/s relative gap=%.2f m start z=%.2f",
		 (double)_param_release_velocity.get(), (double)_param_release_gap.get(), (double)_detach_start_z);
	publishStatus(true);
}

void CustomActionControl::enterAttitudeRecover(hrt_abstime now)
{
	_state = custom_action_status_s::STATE_ATTITUDE_RECOVER;
	_release_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	PX4_INFO("[ATTITUDE_RECOVER] full attitude + XY hold; preload=%.3f, stable time=%.2f s",
		 (double)_release_hold_offset, (double)_param_attitude_recovery_time.get());
	publishStatus(true);
}

bool CustomActionControl::startDetach(uint8_t reason, uint16_t request_id,
		uint8_t source_system, uint16_t source_component)
{
	if (_owner != custom_action_status_s::OWNER_CUSTOM || !localStateValid()) {
		return false;
	}

	if (isDetachState()) {
		return true;
	}

	const hrt_abstime now = hrt_absolute_time();
	_active_request_id = request_id == 0 ? _active_request_id : request_id;
	_active_source_system = source_system == 0 ? _active_source_system : source_system;
	_active_source_component = source_component == 0 ? _active_source_component : source_component;
	_detach_reason = reason;
	_reason = reason;
	_detach_start_z = _local_position.z;
	_hold_x = _local_position.x;
	_hold_y = _local_position.y;
	_hold_z = _local_position.z;
	_locked_yaw = _local_position.heading;
	_clearance_confirm_started = 0;
	_contact_lost_frames = 0;

	if (_state == custom_action_status_s::STATE_CONTACT_PRESS) {
		_release_start_offset = pressureOffset(now);
		_release_hold_offset = math::constrain(_param_release_hold.get(), 0.f, _release_start_offset);
		enterPressRelease(now);

	} else {
		_release_hold_offset = 0.f;
		enterClearanceDescend(now);
	}

	return true;
}

void CustomActionControl::completeDetach()
{
	publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			static_cast<uint8_t>(Result::DetachComplete), _active_request_id,
			_active_source_system, _active_source_component);
	beginHandover(_detach_reason, _active_request_id,
		      _active_source_system, _active_source_component, true);
}

void CustomActionControl::handleDirectionIntent(const vehicle_command_s &command, uint16_t request_id)
{
	if (_owner == custom_action_status_s::OWNER_CUSTOM) {
		PX4_INFO("[CUSTOM] safe detach requested by direction=%d", (int)lroundf(command.param2));

		if (startDetach(custom_action_status_s::REASON_DIRECTION, request_id,
				command.source_system, command.source_component)) {
			publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
				   static_cast<uint8_t>(Result::DetachStarted));

		} else {
			publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
				   static_cast<uint8_t>(Result::None));
		}

	} else if (_owner == custom_action_status_s::OWNER_HANDOVER) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::HandoverPending));

	} else if (_owner == custom_action_status_s::OWNER_LEGACY) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			   static_cast<uint8_t>(Result::LegacyAllowed));

	} else {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::None));
	}
}

void CustomActionControl::handleDetachTop(const vehicle_command_s &command, uint16_t request_id)
{
	uint8_t reason = static_cast<uint8_t>(lroundf(command.param2));

	if (reason == custom_action_status_s::REASON_NONE || reason > custom_action_status_s::REASON_CONTACT_LOST) {
		reason = custom_action_status_s::REASON_DIRECTION;
	}

	if (_owner == custom_action_status_s::OWNER_CUSTOM
	    && startDetach(reason, request_id, command.source_system, command.source_component)) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			   static_cast<uint8_t>(Result::DetachStarted));

	} else if (_owner == custom_action_status_s::OWNER_HANDOVER) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::HandoverPending));

	} else if (_owner == custom_action_status_s::OWNER_LEGACY) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			   static_cast<uint8_t>(Result::LegacyAllowed));

	} else {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::None));
	}
}

void CustomActionControl::handleRebaseComplete(const vehicle_command_s &command, uint16_t request_id)
{
	const uint16_t supplied_handover_id = static_cast<uint16_t>(lroundf(command.param2));

	if (_owner != custom_action_status_s::OWNER_HANDOVER || supplied_handover_id != _handover_id) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::HandoverPending));
		return;
	}

	_owner = custom_action_status_s::OWNER_LEGACY;
	_reason = custom_action_status_s::REASON_NONE;
	_handover_id = 0;
	_handover_started = 0;
	publishStatus(true);
	publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			static_cast<uint8_t>(Result::RebaseAccepted), request_id,
			command.source_system, command.source_component);
	PX4_INFO("[CUSTOM] V3 rebase accepted");
}

void CustomActionControl::handleCommand(const vehicle_command_s &command)
{
	if (command.command == vehicle_command_s::VEHICLE_CMD_NAV_LAND) {
		const bool own_land_request = _land_command_sent && !command.from_external
			&& command.source_system == _vehicle_status.system_id
			&& command.source_component == _vehicle_status.component_id;

		if (own_land_request) {
			return;
		}

		if (_owner == custom_action_status_s::OWNER_CUSTOM
		    || _owner == custom_action_status_s::OWNER_HANDOVER) {
			takeCommanderOwnership(custom_action_status_s::REASON_LAND);
		}

		return;
	}

	if (command.command != custom_action_protocol::kMavCmdUser1
	    || command.target_component != custom_action_protocol::kComponentId) {
		return;
	}

	const int action = lroundf(command.param1);
	const uint16_t request_id = static_cast<uint16_t>(lroundf(command.param3));

	switch (static_cast<Command>(action)) {
	case Command::SearchTop:
		startSearchTop(command, request_id);
		break;

	case Command::DirectionIntent:
		handleDirectionIntent(command, request_id);
		break;

	case Command::RebaseComplete:
		handleRebaseComplete(command, request_id);
		break;

	case Command::DetachTop:
		handleDetachTop(command, request_id);
		break;

	default:
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED,
			   static_cast<uint8_t>(Result::None));
		break;
	}
}

void CustomActionControl::enterTopApproach()
{
	_state = custom_action_status_s::STATE_TOP_APPROACH;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_verify_started = 0;
	_verify_min_distance = NAN;
	_verify_max_distance = NAN;
	_contact_threshold_frames = 0;
	PX4_INFO("[TOP_FOUND] raw=%.3f filtered=%.3f m; slow approach",
		 (double)minimumTopDistance(), (double)_filtered_top_distance);
	publishStatus(true);
}

void CustomActionControl::enterContactVerify(hrt_abstime now)
{
	_state = custom_action_status_s::STATE_CONTACT_VERIFY;
	_verify_started = now;
	_verify_min_distance = _filtered_contact_distance;
	_verify_max_distance = _filtered_contact_distance;
	_contact_threshold_frames = 0;
	PX4_INFO("[CONTACT_VERIFY] close=%u median=%.3f m after %u consecutive frames",
		 (unsigned)closeSensorCount(_param_contact_distance.get()),
		 (double)_filtered_contact_distance, (unsigned)kContactThresholdFramesRequired);
	publishStatus(true);
}

void CustomActionControl::enterContactPress(hrt_abstime now)
{
	if (!captureHoldPoint()) {
		beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
				      _active_source_system, _active_source_component, true);
		return;
	}

	_state = custom_action_status_s::STATE_CONTACT_PRESS;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_reason = custom_action_status_s::REASON_TOP_DISTANCE;
	_verify_started = 0;
	_press_started = now;
	_release_started = 0;
	_clearance_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	_contact_threshold_frames = 0;
	_contact_lost_frames = 0;
	PX4_INFO("[CONTACT_PRESS] stable contact at z=%.2f; pressure ramp active", (double)_hold_z);
	publishStatus(true);
	publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			static_cast<uint8_t>(Result::ContactPressEntered), _active_request_id,
			_active_source_system, _active_source_component);
}

void CustomActionControl::releaseToLegacy(uint8_t reason)
{
	_state = custom_action_status_s::STATE_INACTIVE;
	_owner = custom_action_status_s::OWNER_LEGACY;
	_reason = reason;
	_handover_id = 0;
	_search_started = 0;
	_handover_started = 0;
	_verify_started = 0;
	_press_started = 0;
	_release_started = 0;
	_clearance_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	_contact_threshold_frames = 0;
	_contact_lost_frames = 0;
	_filtered_top_distance = NAN;
	_filtered_contact_distance = NAN;
	_release_start_offset = 0.f;
	_release_hold_offset = 0.f;

	for (float &distance : _detach_start_top_distance) {
		distance = NAN;
	}

	_land_command_last_sent = 0;
	_land_command_sent = false;
	_land_complete_notified = false;
	_detach_reason = custom_action_status_s::REASON_NONE;
	publishStatus(true);
}

void CustomActionControl::takeCommanderOwnership(uint8_t reason)
{
	_state = custom_action_status_s::STATE_INACTIVE;
	_owner = custom_action_status_s::OWNER_COMMANDER;
	_reason = reason;
	_handover_id = 0;
	_search_started = 0;
	_handover_started = 0;
	_verify_started = 0;
	_press_started = 0;
	_release_started = 0;
	_clearance_started = 0;
	_clearance_confirm_started = 0;
	_recover_started = 0;
	_contact_threshold_frames = 0;
	_contact_lost_frames = 0;
	_filtered_top_distance = NAN;
	_filtered_contact_distance = NAN;
	_release_start_offset = 0.f;
	_release_hold_offset = 0.f;

	for (float &distance : _detach_start_top_distance) {
		distance = NAN;
	}

	_land_command_last_sent = 0;
	_land_command_sent = false;
	_land_complete_notified = false;
	_detach_reason = custom_action_status_s::REASON_NONE;
	PX4_INFO("[CUSTOM] control released to Commander: %s", reasonName(reason));
	publishStatus(true);
}

void CustomActionControl::publishControlSetpoint(hrt_abstime now)
{
	const bool precontact = isPrecontactState();
	const bool contact_press = _state == custom_action_status_s::STATE_CONTACT_PRESS;
	const bool press_release = _state == custom_action_status_s::STATE_PRESS_RELEASE;
	const bool clearance_descent = _state == custom_action_status_s::STATE_CLEARANCE_DESCEND;
	const bool attitude_recover = _state == custom_action_status_s::STATE_ATTITUDE_RECOVER;
	const bool handover_hold = _owner == custom_action_status_s::OWNER_HANDOVER;
	offboard_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.position = precontact || clearance_descent || attitude_recover || handover_hold;
	control_mode.velocity = precontact || contact_press || press_release || clearance_descent || attitude_recover;
	control_mode.acceleration = contact_press || press_release || clearance_descent || attitude_recover;
	_offboard_control_mode_pub.publish(control_mode);

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;

	for (int i = 0; i < 3; ++i) {
		setpoint.position[i] = NAN;
		setpoint.velocity[i] = NAN;
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
	}

	if (handover_hold) {
		setpoint.position[0] = _hold_x;
		setpoint.position[1] = _hold_y;
		setpoint.position[2] = _hold_z;

	} else if (precontact) {
		setpoint.position[0] = _hold_x;
		setpoint.position[1] = _hold_y;
		setpoint.velocity[2] = -activeClimbVelocity();

	} else if (contact_press || press_release) {
		setpoint.velocity[2] = 0.f;
		setpoint.acceleration[0] = 0.f;
		setpoint.acceleration[1] = 0.f;
		float thrust_offset = pressureOffset(now);

		if (press_release && _release_started != 0 && now >= _release_started) {
			const float elapsed = static_cast<float>(now - _release_started) * 1e-6f;
			thrust_offset = math::max(_release_hold_offset,
						  _release_start_offset - _param_release_ramp.get() * elapsed);
		}

		const float hover_thrust = math::max(_param_hover_thrust.get(), 0.05f);
		setpoint.acceleration[2] = -CONSTANTS_ONE_G * thrust_offset / hover_thrust;

	} else if (attitude_recover) {
		setpoint.position[0] = _hold_x;
		setpoint.position[1] = _hold_y;
		setpoint.velocity[2] = 0.f;
		const float hover_thrust = math::max(_param_hover_thrust.get(), 0.05f);
		setpoint.acceleration[2] = -CONSTANTS_ONE_G * _release_hold_offset / hover_thrust;

	} else if (clearance_descent) {
		setpoint.position[0] = _hold_x;
		setpoint.position[1] = _hold_y;
		setpoint.velocity[2] = _param_release_velocity.get();
	}

	setpoint.yaw = _locked_yaw;
	setpoint.yawspeed = NAN;
	_trajectory_setpoint_pub.publish(setpoint);
}

void CustomActionControl::publishStatus(bool force)
{
	const hrt_abstime now = hrt_absolute_time();

	if (!force && now - _last_status_publish < kStatusInterval) {
		return;
	}

	custom_action_status_s status{};
	status.timestamp = now;
	status.state = _state;
	status.active = _state != custom_action_status_s::STATE_INACTIVE;
	status.action = isDetachState() ? custom_action_status_s::ACTION_DETACH_TOP
			: (status.active ? custom_action_status_s::ACTION_SEARCH_TOP : custom_action_status_s::ACTION_NONE);
	status.control_owner = _owner;
	status.handover_id = _handover_id;
	status.reason = _reason;
	_status_pub.publish(status);
	_last_status_publish = now;
}

void CustomActionControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	updateParams();
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_local_position_sub.update(&_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
	const bool top_distance_updated = _top_distance_sub.update(&_top_distance);

	vehicle_command_s command{};

	for (int i = 0; i < vehicle_command_s::ORB_QUEUE_LENGTH && _vehicle_command_sub.update(&command); ++i) {
		handleCommand(command);
	}

	// Commands can create state timestamps using hrt_absolute_time(). Capture the
	// cycle time afterwards so elapsed-time calculations can never start from an
	// older timestamp and underflow hrt_abstime.
	const hrt_abstime now = hrt_absolute_time();

	const bool armed = _vehicle_status.timestamp != 0
			   && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (_owner == custom_action_status_s::OWNER_COMMANDER) {
		if (!armed) {
			releaseToLegacy(custom_action_status_s::REASON_NONE);
		}

		publishStatus();
		return;
	}

	if ((_owner == custom_action_status_s::OWNER_CUSTOM
	     || _owner == custom_action_status_s::OWNER_HANDOVER)
	    && (!armed || _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
		if (_land_command_sent
		    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
			takeCommanderOwnership(custom_action_status_s::REASON_LAND);

		} else {
			releaseToLegacy(custom_action_status_s::REASON_NONE);
		}
	}

	bool new_distance_frame = false;

	if (top_distance_updated && _top_distance.sequence != _last_top_distance_sequence) {
		_last_top_distance_sequence = _top_distance.sequence;
		updateFilteredTopDistance();
		new_distance_frame = true;
	}

	if (isPrecontactState()) {
		if (!localStateValid()) {
			beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (_local_position.heading_reset_counter != _heading_reset_counter) {
			startDetach(custom_action_status_s::REASON_HEADING_RESET, _active_request_id,
				    _active_source_system, _active_source_component);

		} else if (!sensorFresh(now)) {
			startDetach(custom_action_status_s::REASON_SENSOR_TIMEOUT, _active_request_id,
				    _active_source_system, _active_source_component);

		} else if (_start_z - _local_position.z >= _param_top_distance.get()) {
			startDetach(custom_action_status_s::REASON_MAX_DISTANCE, _active_request_id,
				    _active_source_system, _active_source_component);

		} else if (_search_started != 0 && now >= _search_started
			   && now - _search_started >= static_cast<hrt_abstime>(_param_top_time.get() * 1_s)) {
			PX4_WARN("[TOP_TIMEOUT] raw=%.3f filtered=%.3f m",
				 (double)minimumTopDistance(), (double)_filtered_top_distance);
			startDetach(custom_action_status_s::REASON_TIMEOUT, _active_request_id,
				    _active_source_system, _active_source_component);

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_SEARCH_TOP
			   && _filtered_top_distance <= _param_top_gap.get()) {
			enterTopApproach();

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_TOP_APPROACH) {
			if (_filtered_top_distance > _param_top_gap.get() + _param_top_hysteresis.get()) {
				_state = custom_action_status_s::STATE_SEARCH_TOP;
				_contact_threshold_frames = 0;
				PX4_INFO("[TOP_APPROACH] top lost; resume search");
				publishStatus(true);

			} else {
				const uint8_t required_sensors = static_cast<uint8_t>(math::constrain(
								     _param_contact_sensor_count.get(), int32_t{1}, int32_t{4}));

				if (closeSensorCount(_param_contact_distance.get()) >= required_sensors) {
					if (_contact_threshold_frames < kContactThresholdFramesRequired) {
						++_contact_threshold_frames;
					}

					if (_contact_threshold_frames >= kContactThresholdFramesRequired) {
						enterContactVerify(now);
					}

				} else {
					_contact_threshold_frames = 0;
				}
			}

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_CONTACT_VERIFY) {
			const uint8_t required_sensors = static_cast<uint8_t>(math::constrain(
							     _param_contact_sensor_count.get(), int32_t{1}, int32_t{4}));

			if (closeSensorCount(_param_contact_distance.get() + _param_top_hysteresis.get()) < required_sensors) {
				enterTopApproach();

			} else {
				_verify_min_distance = math::min(_verify_min_distance, _filtered_contact_distance);
				_verify_max_distance = math::max(_verify_max_distance, _filtered_contact_distance);

				if (_verify_max_distance - _verify_min_distance > _param_stability_band.get()) {
					_verify_started = now;
					_verify_min_distance = _filtered_contact_distance;
					_verify_max_distance = _filtered_contact_distance;

				} else if (_verify_started != 0 && now >= _verify_started
					   && now - _verify_started >= static_cast<hrt_abstime>(_param_contact_time.get() * 1_s)) {
					enterContactPress(now);
				}
			}
		}
	}

	if (_state == custom_action_status_s::STATE_CONTACT_PRESS) {
		if (!localStateValid()) {
			beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (!sensorFresh(now)) {
			startDetach(custom_action_status_s::REASON_SENSOR_TIMEOUT, _active_request_id,
				    _active_source_system, _active_source_component);

		} else if (new_distance_frame) {
			if (minimumTopDistance() > _param_contact_distance.get() + _param_top_hysteresis.get()) {
				if (_contact_lost_frames < kContactLostFramesRequired) {
					++_contact_lost_frames;
				}

				if (_contact_lost_frames >= kContactLostFramesRequired) {
					startDetach(custom_action_status_s::REASON_CONTACT_LOST, _active_request_id,
						    _active_source_system, _active_source_component);
				}

			} else {
				_contact_lost_frames = 0;
			}
		}
	}

	if (_state == custom_action_status_s::STATE_PRESS_RELEASE) {
		const float elapsed = (_release_started != 0 && now >= _release_started)
				      ? static_cast<float>(now - _release_started) * 1e-6f : 0.f;

		if (_release_start_offset - _param_release_ramp.get() * elapsed <= _release_hold_offset) {
			const uint8_t required_sensors = static_cast<uint8_t>(math::constrain(
							     _param_contact_sensor_count.get(), int32_t{1}, int32_t{4}));
			const bool contact_retained = sensorFresh(now)
				&& closeSensorCount(_param_contact_distance.get() + _param_top_hysteresis.get()) >= required_sensors;

			if (contact_retained && _release_hold_offset > 0.f) {
				enterAttitudeRecover(now);

			} else {
				enterClearanceDescend(now);
			}
		}
	}

	if (_state == custom_action_status_s::STATE_ATTITUDE_RECOVER) {
		const uint8_t required_sensors = static_cast<uint8_t>(math::constrain(
						     _param_contact_sensor_count.get(), int32_t{1}, int32_t{4}));
		const bool contact_retained = sensorFresh(now)
			&& closeSensorCount(_param_contact_distance.get() + _param_top_hysteresis.get()) >= required_sensors;

		if (!localStateValid()) {
			beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (!contact_retained) {
			PX4_WARN("[ATTITUDE_RECOVER] contact no longer retained; descend now");
			enterClearanceDescend(now);

		} else if (attitudeBalanced(now)) {
			if (_recover_started == 0) {
				_recover_started = now;
			}

			if (now >= _recover_started
			    && now - _recover_started >= static_cast<hrt_abstime>(_param_attitude_recovery_time.get() * 1_s)) {
				enterClearanceDescend(now);
			}

		} else {
			_recover_started = 0;
		}
	}

	if (_state == custom_action_status_s::STATE_CLEARANCE_DESCEND) {
		if (!localStateValid()) {
			if (_detach_reason == custom_action_status_s::REASON_LAND) {
				requestLand(now, true);

			} else {
				beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
					      _active_source_system, _active_source_component, true);
			}

		} else {
			const float release_gap = math::max(_param_release_gap.get(), 0.01f);
			const float forced_gap = 2.f * release_gap;
			const float local_descent = math::max(_local_position.z - _detach_start_z, 0.f);
			const uint8_t required_sensors = static_cast<uint8_t>(math::constrain(
							     _param_contact_sensor_count.get(), int32_t{1}, int32_t{4}));
			const bool distance_fresh = sensorFresh(now);
			const bool laser_clear = distance_fresh && releasedSensorCount(release_gap) >= required_sensors;
			const bool laser_forced = distance_fresh && releasedSensorCount(forced_gap) >= required_sensors;
			const bool safe_clearance = laser_clear || local_descent >= release_gap;
			const bool forced_clearance = laser_forced || local_descent >= forced_gap;

			if (_detach_reason == custom_action_status_s::REASON_LAND && _land_command_sent) {
				requestLand(now, forced_clearance);

			} else if (safe_clearance) {
				if (_clearance_confirm_started == 0) {
					_clearance_confirm_started = now;
				}

				if (forced_clearance || (now >= _clearance_confirm_started
				    && now - _clearance_confirm_started >= static_cast<hrt_abstime>(_param_release_time.get() * 1_s))) {
					if (_detach_reason == custom_action_status_s::REASON_LAND) {
						requestLand(now, forced_clearance);

					} else {
						completeDetach();
					}
				}

			} else {
				_clearance_confirm_started = 0;
			}
		}
	}

	const hrt_abstime handover_elapsed = (_handover_started != 0 && now >= _handover_started)
					     ? now - _handover_started
					     : 0;
	const bool handover_output_allowed = _owner != custom_action_status_s::OWNER_HANDOVER
		|| handover_elapsed < static_cast<hrt_abstime>(_param_handover_timeout.get() * 1_s);

	if ((_owner == custom_action_status_s::OWNER_CUSTOM
	     || (_owner == custom_action_status_s::OWNER_HANDOVER && handover_output_allowed))
	    && armed && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		publishControlSetpoint(now);
	}

	publishStatus();
}

int CustomActionControl::publishTestCommand(int action, int value, int request_id)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = custom_action_protocol::kMavCmdUser1;
	command.param1 = static_cast<float>(action);
	command.param2 = static_cast<float>(value);
	command.param3 = static_cast<float>(request_id);
	command.target_system = 1;
	command.target_component = custom_action_protocol::kComponentId;
	command.source_system = 42;
	command.source_component = 191;
	command.from_external = true;
	uORB::Publication<vehicle_command_s> publication{ORB_ID(vehicle_command)};
	publication.publish(command);
	PX4_WARN("TEST ONLY command action=%d value=%d request=%d", action, value, request_id);
	return PX4_OK;
}

int CustomActionControl::task_spawn(int argc, char *argv[])
{
	CustomActionControl *instance = new CustomActionControl();

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

int CustomActionControl::custom_command(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[0], "test_command")) {
		if (!strcmp(argv[1], "search") && argc >= 3) {
			return publishTestCommand(static_cast<int>(Command::SearchTop), 0, atoi(argv[2]));
		}

		if (!strcmp(argv[1], "direction") && argc >= 4) {
			return publishTestCommand(static_cast<int>(Command::DirectionIntent), atoi(argv[2]), atoi(argv[3]));
		}

		if (!strcmp(argv[1], "rebase") && argc >= 4) {
			return publishTestCommand(static_cast<int>(Command::RebaseComplete), atoi(argv[2]), atoi(argv[3]));
		}

		if (!strcmp(argv[1], "detach") && argc >= 4) {
			return publishTestCommand(static_cast<int>(Command::DetachTop), atoi(argv[2]), atoi(argv[3]));
		}

		return print_usage("test_command: search <request>, direction <id> <request>, detach <reason> <request>, or rebase <handover> <request>");
	}

	return print_usage("unknown command");
}

int CustomActionControl::print_status()
{
	PX4_INFO("state=%u owner=%u reason=%u handover_id=%u top_seq=%u mask=0x%02x",
		 _state, _owner, _reason, _handover_id, _top_distance.sequence, _top_distance.valid_mask);
	PX4_INFO("hold xyz=(%.2f, %.2f, %.2f) yaw=%.2f",
		 (double)_hold_x, (double)_hold_y, (double)_hold_z, (double)_locked_yaw);
	return 0;
}

int CustomActionControl::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_NAME("custom_action_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_command", "TEST ONLY: inject project command on vehicle_command");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_action_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomActionControl::desc, argc, argv);
}

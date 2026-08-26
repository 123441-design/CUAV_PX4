/****************************************************************************
 * SEARCH_TOP controller implementation.
 ****************************************************************************/

#include "CustomActionControl.hpp"

#include <geo/geo.h>
#include <lib/custom_action_protocol/CustomActionProtocol.hpp>
#include <mathlib/mathlib.h>
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

void CustomActionControl::updateFilteredTopDistance()
{
	const float measured_distance = minimumTopDistance();

	if (!PX4_ISFINITE(measured_distance)) {
		return;
	}

	if (!PX4_ISFINITE(_filtered_top_distance)) {
		_filtered_top_distance = measured_distance;

	} else {
		_filtered_top_distance += _param_top_filter.get() * (measured_distance - _filtered_top_distance);
	}
}

bool CustomActionControl::isPrecontactState() const
{
	return _state == custom_action_status_s::STATE_SEARCH_TOP
	       || _state == custom_action_status_s::STATE_TOP_APPROACH
	       || _state == custom_action_status_s::STATE_CONTACT_VERIFY;
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
	_verify_min_distance = NAN;
	_verify_max_distance = NAN;
	_verify_started = 0;
	_press_started = 0;
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

void CustomActionControl::handleDirectionIntent(const vehicle_command_s &command, uint16_t request_id)
{
	if (_owner == custom_action_status_s::OWNER_CUSTOM) {
		PX4_INFO("[SEARCH_TOP] cancelled by direction=%d", (int)lroundf(command.param2));
		beginHandover(custom_action_status_s::REASON_DIRECTION, request_id,
			      command.source_system, command.source_component, false);
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			   static_cast<uint8_t>(Result::ButtonConsumed));

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
	PX4_INFO("[TOP_FOUND] distance=%.3f m; slow approach", (double)_filtered_top_distance);
	publishStatus(true);
}

void CustomActionControl::enterContactVerify(hrt_abstime now)
{
	_state = custom_action_status_s::STATE_CONTACT_VERIFY;
	_verify_started = now;
	_verify_min_distance = _filtered_top_distance;
	_verify_max_distance = _filtered_top_distance;
	PX4_INFO("[CONTACT_VERIFY] distance=%.3f m", (double)_filtered_top_distance);
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
	_filtered_top_distance = NAN;
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
	_filtered_top_distance = NAN;
	PX4_INFO("[CUSTOM] control released to Commander: %s", reasonName(reason));
	publishStatus(true);
}

void CustomActionControl::publishControlSetpoint(hrt_abstime now)
{
	const bool precontact = isPrecontactState();
	const bool contact_press = _state == custom_action_status_s::STATE_CONTACT_PRESS;
	offboard_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.position = true;
	control_mode.velocity = precontact;
	control_mode.acceleration = contact_press;
	_offboard_control_mode_pub.publish(control_mode);

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.position[0] = _hold_x;
	setpoint.position[1] = _hold_y;
	setpoint.position[2] = (precontact || contact_press) ? NAN : _hold_z;
	setpoint.velocity[0] = NAN;
	setpoint.velocity[1] = NAN;
	setpoint.velocity[2] = precontact ? -activeClimbVelocity() : NAN;

	for (int i = 0; i < 3; ++i) {
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
	}

	if (contact_press) {
		const float elapsed = static_cast<float>(now - _press_started) * 1e-6f;
		const float thrust_offset = math::min(_param_press_add.get(), _param_press_ramp.get() * elapsed);
		const float hover_thrust = math::max(_param_hover_thrust.get(), 0.05f);
		// Mixed-axis PositionControl input: x/y remain position-controlled while z
		// is a finite acceleration feed-forward. This retains attitude control and
		// avoids an unreachable vertical position error and integrator wind-up.
		setpoint.acceleration[2] = -CONSTANTS_ONE_G * thrust_offset / hover_thrust;
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
	status.action = status.active ? custom_action_status_s::ACTION_SEARCH_TOP : custom_action_status_s::ACTION_NONE;
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
	const hrt_abstime now = hrt_absolute_time();
	_vehicle_local_position_sub.update(&_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
	const bool top_distance_updated = _top_distance_sub.update(&_top_distance);

	vehicle_command_s command{};

	for (int i = 0; i < vehicle_command_s::ORB_QUEUE_LENGTH && _vehicle_command_sub.update(&command); ++i) {
		handleCommand(command);
	}

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
		releaseToLegacy(custom_action_status_s::REASON_NONE);
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
			beginHandover(custom_action_status_s::REASON_HEADING_RESET, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (!sensorFresh(now)) {
			beginHandover(custom_action_status_s::REASON_SENSOR_TIMEOUT, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (_start_z - _local_position.z >= _param_top_distance.get()) {
			beginHandover(custom_action_status_s::REASON_MAX_DISTANCE, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (now - _search_started >= static_cast<hrt_abstime>(_param_top_time.get() * 1_s)) {
			beginHandover(custom_action_status_s::REASON_TIMEOUT, _active_request_id,
				      _active_source_system, _active_source_component, true);

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_SEARCH_TOP
			   && _filtered_top_distance <= _param_top_gap.get()) {
			enterTopApproach();

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_TOP_APPROACH) {
			if (_filtered_top_distance > _param_top_gap.get() + _param_top_hysteresis.get()) {
				_state = custom_action_status_s::STATE_SEARCH_TOP;
				PX4_INFO("[TOP_APPROACH] top lost; resume search");
				publishStatus(true);

			} else if (_filtered_top_distance <= _param_contact_distance.get()) {
				enterContactVerify(now);
			}

		} else if (new_distance_frame && _state == custom_action_status_s::STATE_CONTACT_VERIFY) {
			if (_filtered_top_distance > _param_contact_distance.get() + _param_top_hysteresis.get()) {
				enterTopApproach();

			} else {
				_verify_min_distance = math::min(_verify_min_distance, _filtered_top_distance);
				_verify_max_distance = math::max(_verify_max_distance, _filtered_top_distance);

				if (_verify_max_distance - _verify_min_distance > _param_stability_band.get()) {
					_verify_started = now;
					_verify_min_distance = _filtered_top_distance;
					_verify_max_distance = _filtered_top_distance;

				} else if (now - _verify_started >= static_cast<hrt_abstime>(_param_contact_time.get() * 1_s)) {
					enterContactPress(now);
				}
			}

		}
	}

	const bool handover_output_allowed = _owner != custom_action_status_s::OWNER_HANDOVER
		|| now - _handover_started < static_cast<hrt_abstime>(_param_handover_timeout.get() * 1_s);

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

		return print_usage("test_command: search <request>, direction <id> <request>, or rebase <handover> <request>");
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

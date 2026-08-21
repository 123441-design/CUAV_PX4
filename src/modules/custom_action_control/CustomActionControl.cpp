/****************************************************************************
 * SEARCH_TOP controller implementation.
 ****************************************************************************/

#include "CustomActionControl.hpp"

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
constexpr uint8_t kRequiredContactSamples = 4;

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
	case custom_action_status_s::REASON_TOP_CONTACT: return "top contact";
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
	return _top_contact.valid && _top_contact.timestamp != 0 && now >= _top_contact.timestamp
	       && now - _top_contact.timestamp <= timeout;
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
	    || !flightStateAllowsCustom() || !sensorFresh(now) || _top_contact.contact || !captureHoldPoint()) {
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
	_last_top_contact_timestamp = _top_contact.timestamp;
	_contact_confirm_count = 0;
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
	_contact_confirm_count = 0;
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

void CustomActionControl::handleSimulateContact(const vehicle_command_s &command, uint16_t request_id)
{
	if (_owner != custom_action_status_s::OWNER_CUSTOM
	    || _state != custom_action_status_s::STATE_SEARCH_TOP
	    || !flightStateAllowsCustom()) {
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
			   static_cast<uint8_t>(Result::None));
		return;
	}

	// TEST ONLY: updateTestContact() will publish fresh contact=true samples at
	// the module's 50 Hz cycle. The normal four-sample confirmation still
	// applies, so this path exercises the same debounce logic as the real input.
	_test_contact_mode.store(1);
	publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
		   static_cast<uint8_t>(Result::ContactSignalAccepted));
	(void)request_id;
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

	case Command::SimulateContact:
		handleSimulateContact(command, request_id);
		break;

	default:
		publishAck(command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED,
			   static_cast<uint8_t>(Result::None));
		break;
	}
}

void CustomActionControl::enterTopHold()
{
	if (!captureHoldPoint()) {
		beginHandover(custom_action_status_s::REASON_ESTIMATOR, _active_request_id,
			      _active_source_system, _active_source_component, true);
		return;
	}

	_state = custom_action_status_s::STATE_TOP_HOLD;
	_owner = custom_action_status_s::OWNER_CUSTOM;
	_reason = custom_action_status_s::REASON_TOP_CONTACT;
	_test_contact_mode.store(-1);
	_contact_confirm_count = 0;
	PX4_INFO("[SEARCH_TOP] top sensor triggered");
	PX4_INFO("[TOP_HOLD] entered z=%.2f", (double)_hold_z);
	publishStatus(true);
	publishAsyncAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
			static_cast<uint8_t>(Result::TopHoldEntered), _active_request_id,
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
	_contact_confirm_count = 0;
	_test_contact_mode.store(-1);
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
	_contact_confirm_count = 0;
	_test_contact_mode.store(-1);
	PX4_INFO("[CUSTOM] control released to Commander: %s", reasonName(reason));
	publishStatus(true);
}

void CustomActionControl::publishControlSetpoint(hrt_abstime now)
{
	offboard_control_mode_s control_mode{};
	control_mode.timestamp = now;
	control_mode.position = true;
	control_mode.velocity = _state == custom_action_status_s::STATE_SEARCH_TOP;
	_offboard_control_mode_pub.publish(control_mode);

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.position[0] = _hold_x;
	setpoint.position[1] = _hold_y;
	setpoint.position[2] = _state == custom_action_status_s::STATE_SEARCH_TOP ? NAN : _hold_z;
	setpoint.velocity[0] = NAN;
	setpoint.velocity[1] = NAN;
	setpoint.velocity[2] = _state == custom_action_status_s::STATE_SEARCH_TOP ? -_param_top_velocity.get() : NAN;

	for (int i = 0; i < 3; ++i) {
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
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

void CustomActionControl::updateTestContact(hrt_abstime now)
{
	const int mode = _test_contact_mode.load();

	if (mode < 0) {
		return;
	}

	top_contact_s contact{};
	contact.timestamp = now;
	contact.valid = mode != 2;
	contact.contact = mode == 1;
	_test_top_contact_pub.publish(contact);
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
	updateTestContact(now);
	_vehicle_local_position_sub.update(&_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
	const bool top_contact_updated = _top_contact_sub.update(&_top_contact);

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

	if (_state == custom_action_status_s::STATE_SEARCH_TOP) {
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

		} else if (top_contact_updated) {
			// Count only fresh, timestamp-advancing samples. A single contact
			// packet is only a candidate; four consecutive valid samples are
			// required before declaring TOP_HOLD.
			if (_top_contact.timestamp <= _last_top_contact_timestamp
			    || !_top_contact.valid || !_top_contact.contact) {
				_contact_confirm_count = 0;

			} else {
				_last_top_contact_timestamp = _top_contact.timestamp;
				_contact_confirm_count++;

				if (_contact_confirm_count >= kRequiredContactSamples) {
					enterTopHold();
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

void CustomActionControl::setTestContact(int mode)
{
	_test_contact_mode.store(mode);
	PX4_WARN("TEST ONLY top_contact mode=%d", mode);
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
	if (argc >= 1 && !strcmp(argv[0], "test_contact")) {
		CustomActionControl *instance = get_instance<CustomActionControl>(desc);

		if (instance == nullptr || argc < 2) {
			return print_usage("start module first; test_contact requires 0, 1, invalid or clear");
		}

		if (!strcmp(argv[1], "0")) {
			instance->setTestContact(0);
		} else if (!strcmp(argv[1], "1")) {
			instance->setTestContact(1);
		} else if (!strcmp(argv[1], "invalid")) {
			instance->setTestContact(2);
		} else if (!strcmp(argv[1], "clear")) {
			instance->setTestContact(-1);
		} else {
			return print_usage("unknown TEST ONLY contact value");
		}

		return PX4_OK;
	}

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
	PX4_INFO("state=%u owner=%u reason=%u handover_id=%u test_contact=%d",
		 _state, _owner, _reason, _handover_id, _test_contact_mode.load());
	PX4_INFO("hold xyz=(%.2f, %.2f, %.2f) yaw=%.2f",
		 (double)_hold_x, (double)_hold_y, (double)_hold_z, (double)_locked_yaw);
	return 0;
}

int CustomActionControl::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_NAME("custom_action_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_contact", "TEST ONLY: 0, 1, invalid or clear");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_command", "TEST ONLY: inject project command on vehicle_command");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_action_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomActionControl::desc, argc, argv);
}

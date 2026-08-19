/****************************************************************************
 * SEARCH_TOP controller. It owns trajectory output only while status reports
 * CUSTOM or HANDOVER; INACTIVE never represents an additional normal hold.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/custom_action_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/top_contact.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

class CustomActionControl : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CustomActionControl();
	~CustomActionControl() override;

	static Descriptor desc;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	void setTestContact(int mode);
	static int publishTestCommand(int action, int value, int request_id);

private:
	void Run() override;
	void handleCommand(const vehicle_command_s &command);
	void startSearchTop(const vehicle_command_s &command, uint16_t request_id);
	void handleDirectionIntent(const vehicle_command_s &command, uint16_t request_id);
	void handleRebaseComplete(const vehicle_command_s &command, uint16_t request_id);
	void enterTopHold();
	void beginHandover(uint8_t reason, uint16_t handover_id, uint8_t target_system,
			   uint16_t target_component, bool notify_pending);
	void releaseToLegacy(uint8_t reason);
	void takeCommanderOwnership(uint8_t reason);
	bool captureHoldPoint();
	bool flightStateAllowsCustom() const;
	bool localStateValid() const;
	bool sensorFresh(hrt_abstime now) const;
	void publishControlSetpoint(hrt_abstime now);
	void publishStatus(bool force = false);
	void publishAck(const vehicle_command_s &command, uint8_t result, uint8_t project_result);
	void publishAsyncAck(uint8_t result, uint8_t project_result, uint16_t request_id,
			     uint8_t target_system, uint16_t target_component);
	void updateTestContact(hrt_abstime now);

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _top_contact_sub{ORB_ID(top_contact)};

	uORB::Publication<custom_action_status_s> _status_pub{ORB_ID(custom_action_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<top_contact_s> _test_top_contact_pub{ORB_ID(top_contact)};

	vehicle_local_position_s _local_position{};
	vehicle_status_s _vehicle_status{};
	top_contact_s _top_contact{};

	uint8_t _state{custom_action_status_s::STATE_INACTIVE};
	uint8_t _owner{custom_action_status_s::OWNER_LEGACY};
	uint8_t _reason{custom_action_status_s::REASON_NONE};
	uint16_t _handover_id{0};
	uint16_t _active_request_id{0};
	uint8_t _active_source_system{0};
	uint16_t _active_source_component{0};

	float _hold_x{0.f};
	float _hold_y{0.f};
	float _hold_z{0.f};
	float _locked_yaw{0.f};
	float _start_z{0.f};
	uint8_t _heading_reset_counter{0};
	hrt_abstime _search_started{0};
	hrt_abstime _contact_started{0};
	hrt_abstime _handover_started{0};
	hrt_abstime _last_status_publish{0};

	px4::atomic<int> _test_contact_mode{-1}; // -1 disabled, 0/1 valid, 2 invalid

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::CUST_TOP_EN>) _param_enabled,
		(ParamFloat<px4::params::CUST_TOP_VEL>) _param_top_velocity,
		(ParamFloat<px4::params::CUST_TOP_DIST>) _param_top_distance,
		(ParamFloat<px4::params::CUST_TOP_TIME>) _param_top_time,
		(ParamFloat<px4::params::CUST_TOP_DBNC>) _param_top_debounce,
		(ParamFloat<px4::params::CUST_SENS_TO>) _param_sensor_timeout,
		(ParamFloat<px4::params::CUST_HO_TIME>) _param_handover_timeout
	)
};

/****************************************************************************
 * SEARCH_TOP controller. It owns trajectory output only while status reports
 * CUSTOM or HANDOVER; INACTIVE never represents an additional normal hold.
 ****************************************************************************/

#pragma once

#include <cmath>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/custom_action_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/top_distance.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
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
	static int publishTestCommand(int action, int value, int request_id);

private:
	void Run() override;
	void handleCommand(const vehicle_command_s &command);
	void startSearchTop(const vehicle_command_s &command, uint16_t request_id);
	void handleDirectionIntent(const vehicle_command_s &command, uint16_t request_id);
	void handleDetachTop(const vehicle_command_s &command, uint16_t request_id);
	void handleRebaseComplete(const vehicle_command_s &command, uint16_t request_id);
	void enterTopApproach();
	void enterContactVerify(hrt_abstime now);
	void enterContactPress(hrt_abstime now);
	bool startDetach(uint8_t reason, uint16_t request_id, uint8_t source_system, uint16_t source_component);
	void enterPressRelease(hrt_abstime now);
	void enterClearanceDescend(hrt_abstime now);
	void enterAttitudeRecover(hrt_abstime now);
	void completeDetach();
	void beginHandover(uint8_t reason, uint16_t handover_id, uint8_t target_system,
			   uint16_t target_component, bool notify_pending);
	void releaseToLegacy(uint8_t reason);
	void takeCommanderOwnership(uint8_t reason);
	bool captureHoldPoint();
	bool flightStateAllowsCustom() const;
	bool localStateValid() const;
	bool sensorFresh(hrt_abstime now) const;
	float minimumTopDistance() const;
	float contactReferenceDistance() const;
	uint8_t closeSensorCount(float threshold) const;
	uint8_t releasedSensorCount(float distance_increase) const;
	void updateFilteredTopDistance();
	bool isPrecontactState() const;
	bool isDetachState() const;
	bool attitudeBalanced(hrt_abstime now) const;
	float activeClimbVelocity() const;
	float pressureOffset(hrt_abstime now) const;
	void requestLand(hrt_abstime now, bool forced);
	void publishControlSetpoint(hrt_abstime now);
	void publishStatus(bool force = false);
	void publishAck(const vehicle_command_s &command, uint8_t result, uint8_t project_result);
	void publishAsyncAck(uint8_t result, uint8_t project_result, uint16_t request_id,
			     uint8_t target_system, uint16_t target_component);

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _top_distance_sub{ORB_ID(top_distance)};

	uORB::Publication<custom_action_status_s> _status_pub{ORB_ID(custom_action_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _local_position{};
	vehicle_status_s _vehicle_status{};
	top_distance_s _top_distance{};

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
	float _filtered_top_distance{NAN};
	float _filtered_contact_distance{NAN};
	float _verify_min_distance{NAN};
	float _verify_max_distance{NAN};
	float _release_start_offset{0.f};
	float _release_hold_offset{0.f};
	float _detach_start_z{0.f};
	float _detach_start_top_distance[4] {NAN, NAN, NAN, NAN};
	uint8_t _contact_threshold_frames{0};
	uint8_t _contact_lost_frames{0};
	uint8_t _heading_reset_counter{0};
	uint8_t _detach_reason{custom_action_status_s::REASON_NONE};
	hrt_abstime _search_started{0};
	uint16_t _last_top_distance_sequence{0};
	hrt_abstime _verify_started{0};
	hrt_abstime _press_started{0};
	hrt_abstime _release_started{0};
	hrt_abstime _clearance_started{0};
	hrt_abstime _clearance_confirm_started{0};
	hrt_abstime _recover_started{0};
	hrt_abstime _land_command_last_sent{0};
	hrt_abstime _handover_started{0};
	hrt_abstime _last_status_publish{0};
	bool _land_command_sent{false};
	bool _land_complete_notified{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::CUST_TOP_EN>) _param_enabled,
		(ParamFloat<px4::params::CUST_TOP_VEL>) _param_top_velocity,
		(ParamFloat<px4::params::CUST_TOP_DIST>) _param_top_distance,
		(ParamFloat<px4::params::CUST_TOP_TIME>) _param_top_time,
		(ParamFloat<px4::params::CUST_TOP_GAP>) _param_top_gap,
		(ParamFloat<px4::params::CUST_TOP_FILT>) _param_top_filter,
		(ParamFloat<px4::params::CUST_TOP_HYST>) _param_top_hysteresis,
		(ParamFloat<px4::params::CUST_APP_VEL>) _param_approach_velocity,
		(ParamFloat<px4::params::CUST_VER_VEL>) _param_verify_velocity,
		(ParamFloat<px4::params::CUST_CNT_DIST>) _param_contact_distance,
		(ParamFloat<px4::params::CUST_CNT_TIME>) _param_contact_time,
		(ParamInt<px4::params::CUST_CNT_NUM>) _param_contact_sensor_count,
		(ParamFloat<px4::params::CUST_STAB_BND>) _param_stability_band,
		(ParamFloat<px4::params::CUST_PRS_ADD>) _param_press_add,
		(ParamFloat<px4::params::CUST_PRS_RAMP>) _param_press_ramp,
		(ParamFloat<px4::params::CUST_REL_RAMP>) _param_release_ramp,
		(ParamFloat<px4::params::CUST_REL_HOLD>) _param_release_hold,
		(ParamFloat<px4::params::CUST_REL_VEL>) _param_release_velocity,
		(ParamFloat<px4::params::CUST_REL_GAP>) _param_release_gap,
		(ParamFloat<px4::params::CUST_REL_TIME>) _param_release_time,
		(ParamFloat<px4::params::CUST_ATT_TIME>) _param_attitude_recovery_time,
		(ParamFloat<px4::params::CUST_BAL_ANG>) _param_balance_angle,
		(ParamFloat<px4::params::CUST_BAL_RATE>) _param_balance_rate,
		(ParamFloat<px4::params::CUST_SENS_TO>) _param_sensor_timeout,
		(ParamFloat<px4::params::CUST_HO_TIME>) _param_handover_timeout,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_hover_thrust
	)
};

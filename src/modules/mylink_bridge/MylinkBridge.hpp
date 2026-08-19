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

#pragma once

#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/custom_action_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

#include <mavlink.h>
#include <mavlink_types.h>

class MylinkBridge : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	MylinkBridge(const char *device, uint32_t baudrate);
	~MylinkBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	enum class GateState : uint8_t {
		Closed,
		CacheOnly,
		Active
	};

	enum EventFlag : uint32_t {
		EventNone = 0,
		EventTakeoff = 1u << 0,
		EventLand = 1u << 1,
		EventSpeed = 1u << 2,
		EventPause = 1u << 3,
		EventContinue = 1u << 4,
		EventRtl = 1u << 5,
		EventMissionStart = 1u << 6,
		EventMotorThrottle = 1u << 7
	};

	void Run() override;
	void readSerial();
	void handleMavlinkMessage(const mavlink_message_t &message);
	void processMavlinkMessage(const mavlink_message_t &message);
	void updateGateAndCachedMessage();
	void clearSessionState();
	bool cacheableMessage(const mavlink_message_t &message) const;
	void cacheMessage(const mavlink_message_t &message);
	void handlePing(const mavlink_message_t &message);
	void handleHeartbeat(const mavlink_message_t &message);
	void handleCommandLong(const mavlink_message_t &message);
	void handleCustomActionCommand(const mavlink_message_t &message,
				       const mavlink_command_long_t &command);
	void handleMotorTestCommand(const mavlink_message_t &message,
				    const mavlink_command_long_t &command);
	void handleOffboardModeCommand(const mavlink_message_t &message,
				       const mavlink_command_long_t &command);
	void handleCommandInt(const mavlink_message_t &message);
	void handleSetPositionTargetLocalNed(const mavlink_message_t &message, GateState state);
	bool decodeLocalNedSetpoint(const mavlink_message_t &message,
				    trajectory_setpoint_s &setpoint,
				    offboard_control_mode_s &control_mode);

	GateState gateState(vehicle_status_s *status = nullptr);
	bool targetOk(uint8_t target_system, uint8_t target_component);
	bool customTargetOk(uint8_t target_system, uint8_t target_component);
	bool legacyControlAllowed();
	bool commandSupported(uint16_t command) const;
	uint32_t eventFlagForCommand(const mavlink_command_long_t &command) const;
	void activateDirectMotorControl(float throttle_percent, uint32_t timeout_ms);
	void updateDirectMotorControl(GateState state);
	void publishDirectMotorSetpoint(float throttle_percent);
	void releaseDirectMotorControl(bool hold_zero = true);

	void sendMavlinkMessage(const mavlink_message_t &message);
	void publishVehicleCommand(const mavlink_message_t &message,
				   const mavlink_command_long_t &command,
				   uint16_t vehicle_command);
	void sendCommandAck(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2,
			    uint8_t target_system, uint8_t target_component);
	void relayVehicleCommandAcks();
	void sendVehicleCommandAck(const vehicle_command_ack_s &ack);

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _custom_action_status_sub{ORB_ID(custom_action_status)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	device::Serial _serial;
	mavlink_message_t _rx_parser_message{};
	mavlink_status_t _rx_parser_status{};
	mavlink_status_t _tx_status{};

	uint32_t _rx_bytes{0};
	uint32_t _valid_frames{0};
	uint32_t _bad_frames{0};
	uint32_t _gate_dropped_frames{0};
	uint32_t _cached_updates{0};
	uint32_t _session_resets{0};
	uint32_t _handled_messages{0};
	uint32_t _unsupported_messages{0};
	uint32_t _offboard_setpoints_received{0};
	uint32_t _offboard_control_mode_published{0};
	uint32_t _trajectory_setpoints_published{0};
	uint32_t _offboard_mode_requests{0};
	uint32_t _custom_action_commands{0};
	uint32_t _legacy_setpoints_blocked{0};
	uint32_t _relayed_command_acks{0};
	uint32_t _invalid_setpoints{0};
	uint32_t _motor_test_commands{0};
	uint32_t _invalid_motor_test_commands{0};
	uint32_t _motor_test_releases{0};
	uint32_t _direct_motor_setpoints_published{0};
	uint32_t _tx_frames{0};
	uint32_t _tx_bytes{0};
	uint32_t _rx_errors{0};
	uint32_t _tx_errors{0};
	int _last_rx_errno{0};
	int _last_tx_errno{0};

	mavlink_message_t _cached_message{};
	mavlink_command_long_t _latest_command{};
	trajectory_setpoint_s _latest_setpoint{};
	bool _cache_valid{false};
	bool _setpoint_cache_valid{false};
	hrt_abstime _last_setpoint_rx{0};
	bool _was_armed{false};
	bool _commander_owns_control{false};
	custom_action_status_s _custom_action_status{};
	uint8_t _remote_system{0};
	uint16_t _remote_component{0};
	uint32_t _event_flags{EventNone};
	uint16_t _latest_event_command{0};
	bool _direct_motor_active{false};
	bool _direct_motor_stopping{false};
	float _motor_throttle_percent{0.f};
	hrt_abstime _motor_test_deadline{0};
	hrt_abstime _motor_stop_deadline{0};
};

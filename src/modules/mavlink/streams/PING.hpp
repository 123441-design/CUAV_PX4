/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef PING_HPP
#define PING_HPP

#include <lib/custom_action_protocol/CustomActionProtocol.hpp>
#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#if defined(__PX4_POSIX)
#include <uORB/topics/actuator_motors.h>
#else
#include <uORB/topics/actuator_outputs.h>
#endif
#include <uORB/topics/custom_action_status.h>
#include <uORB/topics/top_distance.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamPing : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamPing(mavlink); }

	static constexpr const char *get_name_static() { return "PING"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PING; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return 4 * (MAVLINK_MSG_ID_PING_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

	bool const_rate() override { return true; }

private:
	explicit MavlinkStreamPing(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _top_distance_sub{ORB_ID(top_distance)};
	uORB::Subscription _custom_action_status_sub{ORB_ID(custom_action_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
#if defined(__PX4_POSIX)
	uORB::Subscription _motor_output_sub{ORB_ID(actuator_motors)};
#else
	uORB::Subscription _motor_output_sub{ORB_ID(actuator_outputs)};
#endif
	uint16_t _motor_sequence{0};
	uint16_t _custom_status_sequence{0};
	uint16_t _pressure_detail_sequence{0};
	uint8_t _standard_ping_divider{0};

	bool send() override
	{
		bool sent = send_top_distance();
		sent = send_motor_outputs() || sent;
		sent = send_custom_action_status() || sent;

		if (++_standard_ping_divider >= 100) {
			_standard_ping_divider = 0;
			mavlink_ping_t message{};
			message.time_usec = hrt_absolute_time();
			mavlink_msg_ping_send_struct(_mavlink->get_channel(), &message);
			sent = true;
		}

		return sent;
	}

	bool send_top_distance()
	{
		top_distance_s report{};

		if (!_top_distance_sub.update(&report) || report.timestamp == 0) {
			return false;
		}

		uint64_t packed = 0;
		uint8_t valid_mask = 0;

		for (uint8_t index = 0; index < custom_action_protocol::kTopDistanceSensorCount; ++index) {
			const float distance_m = report.distance_m[index];
			const uint8_t sensor_bit = 1u << index;

			if ((report.valid_mask & sensor_bit) != 0 && PX4_ISFINITE(distance_m)
			    && distance_m > 0.f && distance_m <= 65.535f) {
				const uint16_t distance_mm = static_cast<uint16_t>(distance_m * 1000.f + 0.5f);
				packed |= static_cast<uint64_t>(distance_mm)
					  << (index * custom_action_protocol::kTopDistanceArrayDistanceBits);
				valid_mask |= sensor_bit;
			}
		}

		mavlink_ping_t message{};
		message.time_usec = packed;
		message.seq = custom_action_protocol::kTopDistanceArrayMarker
			      | custom_action_protocol::kTopDistanceArrayVersion
			      | (static_cast<uint32_t>(valid_mask) << custom_action_protocol::kTopDistanceArrayValidShift)
			      | (static_cast<uint32_t>(report.sequence)
				 & custom_action_protocol::kTopDistanceArraySequenceMask);
		send_to_upper_computer(message);
		return true;
	}

	bool send_custom_action_status()
	{
		custom_action_status_s status{};

		if (!_custom_action_status_sub.update(&status) || status.timestamp == 0) {
			return false;
		}

		mavlink_ping_t message{};
		message.time_usec = status.handover_id;
		message.seq = custom_action_protocol::kCustomStatusMarker
			      | custom_action_protocol::kCustomStatusVersion
			      | ((static_cast<uint32_t>(status.state) << custom_action_protocol::kCustomStatusStateShift)
				 & custom_action_protocol::kCustomStatusStateMask)
			      | ((static_cast<uint32_t>(status.control_owner) << custom_action_protocol::kCustomStatusOwnerShift)
				 & custom_action_protocol::kCustomStatusOwnerMask)
			      | (status.active ? custom_action_protocol::kCustomStatusActiveFlag : 0u)
			      | ((static_cast<uint32_t>(status.reason) << custom_action_protocol::kCustomStatusReasonShift)
				 & custom_action_protocol::kCustomStatusReasonMask)
			      | (static_cast<uint32_t>(++_custom_status_sequence)
				 & custom_action_protocol::kCustomStatusSequenceMask);
		send_to_upper_computer(message);
		send_pressure_detail(status);
		return true;
	}

	void send_pressure_detail(const custom_action_status_s &status)
	{
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

		mavlink_ping_t message{};
		message.time_usec = packed_pressure;
		message.seq = custom_action_protocol::kPressureDetailMarker
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
		send_to_upper_computer(message);
	}

	bool send_motor_outputs()
	{
		vehicle_status_s status{};

		if (!_vehicle_status_sub.copy(&status) || status.timestamp == 0) {
			return false;
		}

		const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		const hrt_abstime now = hrt_absolute_time();
		uint64_t packed = 0;
		uint8_t valid_mask = 0;

#if defined(__PX4_POSIX)
		// SITL has no physical PWM pins. Convert the logical motor command to a
		// conventional PWM-equivalent value so the same UI and protocol can be
		// exercised without presenting it as measured hardware feedback.
		actuator_motors_s outputs{};
		const bool fresh = _motor_output_sub.copy(&outputs) && outputs.timestamp != 0
				   && now >= outputs.timestamp && now - outputs.timestamp <= 500000;
#else
		actuator_outputs_s outputs{};
		const bool fresh = _motor_output_sub.copy(&outputs) && outputs.timestamp != 0
				   && now >= outputs.timestamp && now - outputs.timestamp <= 500000;

#if defined(CONFIG_ARCH_BOARD_CUAV_FMU_V6X)
		// Current aircraft output functions:
		// MAIN1=M4, MAIN2=M3, MAIN3=M1, MAIN4=M2.
		static constexpr uint8_t output_by_motor[custom_action_protocol::kMotorOutputCount] {2, 3, 1, 0};
#else
		static constexpr uint8_t output_by_motor[custom_action_protocol::kMotorOutputCount] {0, 1, 2, 3};
#endif
#endif

		for (uint8_t index = 0; index < custom_action_protocol::kMotorOutputCount; ++index) {
			uint16_t pwm_us = 0;
			bool valid = false;

#if defined(__PX4_POSIX)
			if (fresh && PX4_ISFINITE(outputs.control[index])) {
				const float normalized = math::constrain(outputs.control[index], 0.f, 1.f);
				pwm_us = static_cast<uint16_t>(custom_action_protocol::kMotorPwmSimMinimumUs
						+ custom_action_protocol::kMotorPwmSimRangeUs * normalized + 0.5f);
				valid = true;
			}
#else
			const uint8_t output_index = output_by_motor[index];

			if (fresh && output_index < outputs.noutputs && PX4_ISFINITE(outputs.output[output_index])) {
				const float output = outputs.output[output_index];
				valid = output >= custom_action_protocol::kMotorPwmMinimumUs
					&& output <= custom_action_protocol::kMotorPwmMaximumUs;
				pwm_us = valid ? static_cast<uint16_t>(output + 0.5f) : 0;
			}
#endif

			if (valid) {
				packed |= static_cast<uint64_t>(pwm_us)
					  << (index * custom_action_protocol::kMotorOutputArrayValueBits);
				valid_mask |= 1u << index;
			}
		}

		mavlink_ping_t message{};
		message.time_usec = packed;
		message.seq = custom_action_protocol::kMotorOutputArrayMarker
			      | custom_action_protocol::kMotorOutputArrayVersion
			      | (static_cast<uint32_t>(valid_mask) << custom_action_protocol::kMotorOutputArrayValidShift)
			      | (armed ? custom_action_protocol::kMotorOutputArrayArmedFlag : 0u)
			      | (static_cast<uint32_t>(++_motor_sequence)
				 & custom_action_protocol::kMotorOutputArraySequenceMask);
		send_to_upper_computer(message);
		return true;
	}

	void send_to_upper_computer(mavlink_ping_t &message)
	{
		message.target_system = custom_action_protocol::kUpperComputerSystemId;
		message.target_component = custom_action_protocol::kUpperComputerComponentId;
		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &message);
	}
};

#endif // PING_HPP

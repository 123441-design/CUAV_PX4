/****************************************************************************
 * Dedicated TELEM1 receiver for one-packet four-laser MAVLink reports.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/top_distance.h>
#include <uORB/topics/vehicle_status.h>

#include <mavlink.h>

class TopDistanceBridge : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	TopDistanceBridge(const char *device, uint32_t baudrate);
	~TopDistanceBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void readSerial();
	void handleMavlinkMessage(const mavlink_message_t &message);

	device::Serial _serial;
	mavlink_message_t _rx_parser_message{};
	mavlink_status_t _rx_parser_status{};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<top_distance_s> _top_distance_pub{ORB_ID(top_distance)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	uint32_t _rx_bytes{0};
	uint32_t _valid_mavlink_frames{0};
	uint32_t _bad_mavlink_frames{0};
	uint32_t _published_reports{0};
	uint32_t _reports_with_invalid_sensors{0};
	uint32_t _rejected_reports{0};
	uint32_t _ignored_messages{0};
	uint32_t _rx_errors{0};
	int _last_rx_errno{0};
};

/****************************************************************************
 * Dedicated TELEM1 receiver for one-packet four-laser MAVLink reports.
 ****************************************************************************/

#include "TopDistanceBridge.hpp"

#include <drivers/drv_hrt.h>
#include <lib/custom_action_protocol/CustomActionProtocol.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

#include <cerrno>
#include <cinttypes>
#include <cmath>

using namespace time_literals;

namespace
{
constexpr hrt_abstime kRunInterval = 4_ms;
constexpr unsigned kMaximumDrainReads = 8;
}

ModuleBase::Descriptor TopDistanceBridge::desc{task_spawn, custom_command, print_usage};

TopDistanceBridge::TopDistanceBridge(const char *device, uint32_t baudrate) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_serial(device != nullptr ? device : "/dev/null", baudrate)
{
}

TopDistanceBridge::~TopDistanceBridge()
{
	ScheduleClear();
	_serial.close();
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool TopDistanceBridge::init()
{
	// Open and use the UART from the same work-queue task group on NuttX.
	ScheduleNow();
	return true;
}

void TopDistanceBridge::handleMavlinkMessage(const mavlink_message_t &message)
{
	if (message.msgid != MAVLINK_MSG_ID_PING) {
		_ignored_messages++;
		return;
	}

	mavlink_ping_t ping{};
	mavlink_msg_ping_decode(&message, &ping);

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);
	const uint8_t system_id = status.system_id > 0 ? status.system_id : 1;

	if (ping.target_component != custom_action_protocol::kComponentId
	    || (ping.target_system != 0 && ping.target_system != system_id)) {
		_rejected_reports++;
		return;
	}

	const uint32_t metadata = ping.seq;

	if ((metadata & custom_action_protocol::kTopDistanceArrayMarkerMask)
	    != custom_action_protocol::kTopDistanceArrayMarker
	    || (metadata & custom_action_protocol::kTopDistanceArrayVersionMask)
	    != custom_action_protocol::kTopDistanceArrayVersion
	    || (metadata & custom_action_protocol::kTopDistanceArrayReservedMask) != 0) {
		_rejected_reports++;
		return;
	}

	const uint8_t requested_valid_mask = static_cast<uint8_t>(
		(metadata & custom_action_protocol::kTopDistanceArrayValidMask)
		>> custom_action_protocol::kTopDistanceArrayValidShift);
	const uint16_t sequence = static_cast<uint16_t>(
		metadata & custom_action_protocol::kTopDistanceArraySequenceMask);
	const uint64_t packed_distances = ping.time_usec;
	const hrt_abstime now = hrt_absolute_time();
	top_distance_s report{};

	for (uint8_t sensor_id = 0;
	     sensor_id < custom_action_protocol::kTopDistanceSensorCount;
	     ++sensor_id) {
		const uint8_t sensor_bit = 1u << sensor_id;
		const uint8_t shift = sensor_id * custom_action_protocol::kTopDistanceArrayDistanceBits;
		const uint16_t distance_mm = static_cast<uint16_t>(
			(packed_distances >> shift) & custom_action_protocol::kTopDistanceArrayMillimetresMask);
		const bool valid = (requested_valid_mask & sensor_bit) != 0 && distance_mm > 0;

		report.timestamp_sample[sensor_id] = now;
		report.distance_m[sensor_id] = valid ? distance_mm * 0.001f : NAN;

		if (valid) {
			report.valid_mask |= sensor_bit;
		}
	}

	report.timestamp = now;
	report.sequence = sequence;
	_top_distance_pub.publish(report);
	_published_reports++;

	const uint8_t all_sensor_mask =
		(1u << custom_action_protocol::kTopDistanceSensorCount) - 1u;

	if (report.valid_mask != all_sensor_mask) {
		_reports_with_invalid_sensors++;
	}
}

void TopDistanceBridge::readSerial()
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
			const uint8_t framing = mavlink_frame_char_buffer(
				&_rx_parser_message, &_rx_parser_status, buffer[index], &message, &status);

			if (framing == MAVLINK_FRAMING_OK) {
				_valid_mavlink_frames++;
				handleMavlinkMessage(message);

			} else if (framing == MAVLINK_FRAMING_BAD_CRC
				   || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
				_bad_mavlink_frames++;
			}
		}
	}
}

void TopDistanceBridge::Run()
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

		PX4_INFO("four-laser MAVLink RX opened %s at %" PRIu32 " baud",
			 _serial.getPort(), _serial.getBaudrate());
		ScheduleOnInterval(kRunInterval);
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
	readSerial();
	perf_end(_loop_perf);
}

int TopDistanceBridge::task_spawn(int argc, char *argv[])
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
		PX4_ERR("a valid serial device is required");
		return PX4_ERROR;
	}

	TopDistanceBridge *instance = new TopDistanceBridge(device, static_cast<uint32_t>(baudrate));

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

int TopDistanceBridge::print_status()
{
	PX4_INFO("transport=UART port=%s baud=%" PRIu32 " MAVLink2",
		 _serial.getPort(), _serial.getBaudrate());
	PX4_INFO("rx: bytes=%" PRIu32 " valid_frames=%" PRIu32 " bad_frames=%" PRIu32
		 " errors=%" PRIu32 " last_errno=%d",
		 _rx_bytes, _valid_mavlink_frames, _bad_mavlink_frames, _rx_errors, _last_rx_errno);
	PX4_INFO("top_distance: published=%" PRIu32 " invalid_sensors=%" PRIu32
		 " rejected=%" PRIu32 " ignored=%" PRIu32,
		 _published_reports, _reports_with_invalid_sensors, _rejected_reports, _ignored_messages);
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int TopDistanceBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TopDistanceBridge::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_NAME("top_distance_bridge", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Four-laser serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int top_distance_bridge_main(int argc, char *argv[])
{
	return ModuleBase::main(TopDistanceBridge::desc, argc, argv);
}

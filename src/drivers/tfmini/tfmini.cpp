/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/**
 * @file tfmini.cpp
 * @author Alessandro Simovic
 *
 * Driver for the Benawake TFMini micro lidar range finder, connected via UART.
 */

#include "tfmini.h"

#include <px4_defines.h>
#include <px4_getopt.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace tfmini
{

extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);

struct work_s TFMini::_work = {};
uint8_t TFMini::_next_instance_id = 0;

TFMini::TFMini(const char *const device, uint8_t orientation):
	CDev("tfmini", TFMINI_DEVICE_PATCH),
	_distance_sensor_topic(nullptr),
	_orb_class_instance(-1)
{
	strncpy(_device_path, device, sizeof(_device_path));
	_device_path[sizeof(_device_path) - 1] = '\0';  // Fix in case of overflow

	// Add static data to sensor message
	sensor_msg.min_distance = TFMINI_MIN_DISTANCE;
	sensor_msg.max_distance = TFMINI_MAX_DISTANCE;
	sensor_msg.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	sensor_msg.orientation = orientation;
	sensor_msg.id = TFMini::_next_instance_id++;
}

TFMini::~TFMini()
{
	::close(_uart_file_des);
}

int TFMini::task_spawn(int argc, char *argv[])
{
	TFMini *tfmini = TFMini::instantiate(argc, argv);

	if (tfmini == nullptr) {
		PX4_ERR("Failed to instantiate module");
		return PX4_ERROR;
	}

	_object = tfmini;

	/* schedule first cycle as soon as possible */
	int ret = work_queue(HPWORK, &_work, (worker_t)&TFMini::cycle_trampoline, tfmini, 0);

	if (ret < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

void TFMini::cycle()
{
	sensor_msg.timestamp = hrt_absolute_time();
	sensor_msg.current_distance = 0;
	sensor_msg.covariance = 0.0f;

	/* publish sensor data if this is the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &sensor_msg);
	}
}

void TFMini::cycle_trampoline(void *arg)
{
	TFMini *tfmini = reinterpret_cast<TFMini *>(arg);
	tfmini->cycle();

	// Schedule next execution
	if (!tfmini->should_exit()) {
		work_queue(HPWORK, &_work, (worker_t)&TFMini::cycle_trampoline, tfmini,
			   USEC2TICK(1000000 / TFMINI_SENSOR_RATE));

	} else {
		exit_and_cleanup();
	}
}

TFMini *TFMini::instantiate(int argc, char *argv[])
{
	// Parse arguments
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	PX4_INFO("argc: %i", argc);

	for (int i = 0; i < argc; i++) {
		PX4_INFO(argv[i]);
	}

	if (argc < 2) {
		print_usage("not enough arguments");
		return nullptr;
	}

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
	}

	/* Sanity check on arguments */
	if (device == nullptr || strlen(device) == 0) {
		print_usage("no device specified");
		return nullptr;
	}

	// Instantiate module
	// TODO: Parse orientation argument
	TFMini *tfmini = new TFMini(device, distance_sensor_s::ROTATION_DOWNWARD_FACING);

	if (tfmini->init() != 0) {
		PX4_ERR("failed to initialize module");
		delete tfmini;
		return nullptr;
	}

	return tfmini;
}

int TFMini::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TFMini::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is a driver for the Benawake TFMini micro lidar sensor.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tfmini", "driver");
	// PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	return PX4_OK;
}

int TFMini::print_status()
{
	//TODO
	return PX4_OK;
}

int TFMini::init()
{
	PX4_INFO("Configuring UART");
	_uart_file_des = TFMiniProto::init(_device_path);
	if(_uart_file_des<0){
		PX4_ERR("failed to open uart device!");
		return PX4_ERROR;
	}

	// Advertise uORB
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor),
	&sensor_msg, &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_WARN("failed to create distance_sensor object. Is uOrb running?");
	}

	return PX4_OK;
}

int TFMiniProto::init(const char* device){
	// Open and configure UART port
	int uart_fd = ::open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (uart_fd < 0) {
		return PX4_ERROR;
	}

	int ret = TFMiniProto::uart_config(uart_fd);

	if (!ret){
		return uart_fd;
	}else{
		return -1;
	}
}

int TFMiniProto::uart_config(int uart_fd){
	int termios_state = -1;
	struct termios config;

	/* Make sure the struct does not have anything in it*/
	memset(&config, 0, sizeof(struct termios));

	/* Get the current configurations of the termios device. */
	if (tcgetattr(uart_fd, &config) != 0)
	{
		PX4_ERR("TFMiniProto: tcgetattr call failed.");
		return PX4_ERROR;
	}

	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
										INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// // Apply the config changes
	if(tcsetattr(uart_fd, TCSANOW, &config) < 0)
	{
		PX4_ERR("TFMiniProto: failed to apply termios configuration: %d\n", termios_state);
		return PX4_ERROR;
	}

	// Set baud rate
	if(cfsetispeed(&config, B115200) < 0 ||
	   cfsetospeed(&config, B115200) < 0) {
		PX4_ERR("TFMiniProto: failed to set baudrate: %d\n", termios_state);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int tfmini_main(int argc, char *argv[])
{
	return TFMini::main(argc, argv);
}

} // namespace tfmini

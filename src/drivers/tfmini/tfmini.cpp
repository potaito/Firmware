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

#include <drivers/device/device.h>
#include <px4_workqueue.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <uORB/topics/distance_sensor.h>

#include <fcntl.h>
#include <termios.h>

#	define BAUD_RATE 115200

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#	define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

#define TFMINI_DEVICE_PATCH "/dev/tfmini"


extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);


class TFMini : public device::CDev, public ModuleBase<TFMini>
{
public:
	TFMini(const char *const device);
	~TFMini();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TFMini *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	void run() override;

	/** @see ModuleBase */
	// int print_status() override;

	static void cycle_trampoline(void *arg);

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

private:
	/** Prevent copies */
	TFMini(const TFMini &other);
	int init();
	char _device_path[DEVICE_ARGUMENT_MAX_LENGTH];

	static struct work_s	_work;
	int _uart_file_des = -1;
};

TFMini::TFMini(const char *const device):
	CDev("tfmini", TFMINI_DEVICE_PATCH)
{
	strncpy(_device_path, device, sizeof(_device_path));
	_device_path[sizeof(_device_path) - 1] = '\0';  // Fix in case of overflow
}

TFMini::~TFMini()
{
	// TODO: uorb unadvertise

	::close(_uart_file_des);
}

int TFMini::task_spawn(int argc, char *argv[])
{
	// Parse thread-flag argument only
	bool run_as_thread = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	if (argc > 2) {
		while ((ch = px4_getopt(argc, argv, "t:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 't':
				run_as_thread = true;
				break;
			}
		}
	}

	if (!run_as_thread) {
		TFMini *dev = TFMini::instantiate(argc, argv);

		if (dev == nullptr) {
			PX4_ERR("Failed to instantiate module");
			return PX4_ERROR;
		}

		if (dev->init() != 0) {
			PX4_ERR("init failed");
			delete dev;
			return PX4_ERROR;
		}

		_object = dev;

		/* schedule a cycle to start things */
		int ret = work_queue(HPWORK, &_work, (worker_t)&TFMini::cycle_trampoline, dev, 0);

		if (ret < 0) {
			return ret;
		}

		_task_id = task_id_is_work_queue;

	} else {

		/* start the IO interface task */

		_task_id = px4_task_spawn_cmd("tfmini",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_SLOW_DRIVER,  // TODO: Not sure about that...
					      1310,
					      (px4_main_t)&run_trampoline,
					      nullptr);

		if (_task_id < 0) {
			_task_id = -1;
			return -errno;
		}
	}

	// wait until task is up & running (the mode_* commands depend on it)
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

void TFMini::cycle()
{
	PX4_INFO("cycle()");
}

void TFMini::cycle_trampoline(void *arg)
{
	TFMini *dev = reinterpret_cast<TFMini *>(arg);
	dev->cycle();
}

/** @see ModuleBase */
TFMini *TFMini::instantiate(int argc, char *argv[])
{
	// Parse arguments
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

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
	}

	// Instantiate module
	TFMini *tfmini = new TFMini(device);

	if (tfmini->init() != 0) {
		PX4_ERR("failed to initialize module");
		delete tfmini;
		return nullptr;
	}

	return tfmini;
}

/** @see ModuleBase */
int TFMini::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

/** @see ModuleBase */
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

/** @see ModuleBase */
void TFMini::run()
{
	// Main loop
	while (!should_exit()) {
		cycle();
	}
}

/** @see ModuleBase */
// int TFMini::print_status()
// {
// 	//TODO
// 	return PX4_OK;
// }

int TFMini::init()
{
	// Open UART port
	// TODO: Set right options
	_uart_file_des = ::open(_device_path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if (_uart_file_des < 0) {
		PX4_ERR("failed to open uart device!");
		return PX4_ERROR;
	}

	// Set UART baudrate
	int speed = BAUD_RATE;
	int termios_state = -1;
	struct termios uart_config;

	// TODO: Lookup online if this is the proper way!
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", _device_path, termios_state);
		return PX4_ERROR;
	}

	return PX4_OK;
}


int tfmini_main(int argc, char *argv[])
{
	// return TFMini::main(argc, argv);

	return 0;
}

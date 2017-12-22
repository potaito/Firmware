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

#pragma once

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


namespace tfmini
{

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
	int print_status() override;

	static void cycle_trampoline(void *arg);

	// int start();

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




} // namespace tfmini

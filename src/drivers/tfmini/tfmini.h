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

#include <drivers/device/ringbuffer.h>
#include <drivers/device/device.h>
#include <px4_workqueue.h>
#include <px4_module.h>
#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#	define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

#define TFMINI_DEVICE_PATCH "/dev/tfmini"
#define TFMINI_BAUD_RATE B115200

#define TFMINI_SENSOR_RATE 100
#define TFMINI_MIN_DISTANCE (0.30f)
#define TFMINI_MAX_DISTANCE (12.00f)

namespace tfmini
{

extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);


class TFMini : public device::CDev, public ModuleBase<TFMini>
{
public:
	TFMini(const char *const device, uint8_t orientation);
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
	int print_status() override;

	static void cycle_trampoline(void *arg);

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

private:
	/** Prevent copies of class */
	TFMini(const TFMini &);
	TFMini operator=(const TFMini &);

	int init();
	char _device_path[DEVICE_ARGUMENT_MAX_LENGTH];
	orb_advert_t		_distance_sensor_topic {};
	int				_orb_class_instance;
	struct distance_sensor_s sensor_msg;
	static struct work_s	_work;
	int _uart_file_des = -1;
	static uint8_t _next_instance_id;
};

struct TFMiniProto {

	static int init(const char *device);

	// Configure UART (baud rate and other options)
	static int uart_config(int uart_fd);
};

} // namespace tfmini

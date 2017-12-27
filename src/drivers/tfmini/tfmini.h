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
 * @file tfmini.h
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

#define TFMINI_FRAME_SIZE 9       // Frame size in bytes
#define TFMINI_FRAME_HEADER 0x59  // Has to appear twice in succession
#define TFMINI_BYTE_POS_DIST_L 2      // Index of Distance low byte
#define TFMINI_BYTE_POS_DIST_H 3      // Index of Distance high byte
#define TFMINI_BYTE_POS_CRC 8         // Index of CRC byte

#define TFMINI_DEVICE_PATCH "/dev/tfmini"
#define TFMINI_BAUD_RATE B115200
#define TFMINI_WAIT_BEFORE_READ_MICRO_SECS  1000  // Given the baud rate and frame size, the theoretical transmission time per frame is 625 microseconds.
#define BUFFER_SIZE 18

#define TFMINI_SENSOR_RATE 100       // Sensor update rate in Hz
#define TFMINI_MIN_DISTANCE (0.30f)  // Minimum sensor distance in meters
#define TFMINI_MAX_DISTANCE (12.00f) // Maximum sensor distance in meters

namespace tfmini
{

extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);

// TODO: Why does it need to be a device? Do I need the features this provides?
class TFMini : public device::CDev, public ModuleBase<TFMini>
{
public:
	/**
	 * Default constructor
	 * @param device UNIX path of UART device where sensor is attached
	 * @param orientation as defined in msg/distance_sensor.msg
	 */
	TFMini(const char *const device, uint8_t orientation);

	/**
	 * Default destructor
	 */
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

	/**
	 * Entry point for the work scheduler
	 * @param arg command line arguments
	 */
	static void cycle_trampoline(void *arg);

	/**
	 * run the main loop
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
	uint8_t *_buffer[BUFFER_SIZE];
};

struct TFMiniProto {
	/**
	 * Open and configure the UART serial device where sensor is attached
	 * @param device UNIX path of device
	 * @returns file descriptor of device, negative means error
	 */
	static int init(const char *device);

	/**
	 * Configure UART (baud rate and other options)
	 * @param uart_fd File descriptor of serial device previously opened
	 */
	static int uart_config(int uart_fd);

	/**
	 * Parse buffer into distance sensor message
	 * @param buffer RC buffer containing raw sensor data
	 * @param buff_len Number of bytes in buffer that are containing data
	 * @param data Distance sensor data that will be filled
	 * @returns true on success, false on failure
	 */
	static bool parse(uint8_t *const buffer, size_t buff_len, distance_sensor_s *const data);
};

} // namespace tfmini

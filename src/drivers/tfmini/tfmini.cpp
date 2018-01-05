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
 */

#include "tfmini.h"

#include <px4_defines.h>
#include <px4_getopt.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace tfmini
{

// static members
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
	PX4_INFO("task_spawn");
	TFMini *tfmini = TFMini::instantiate(argc, argv);

	if (tfmini == nullptr) {
		PX4_ERR("Failed to instantiate module");
		return PX4_ERROR;
	}

	_object = tfmini;

	/* schedule first cycle as soon as possible */
	PX4_INFO("Queuing first cycle");
	int ret = work_queue(HPWORK, &_work, (worker_t)&TFMini::cycle_trampoline, tfmini, 0);

	if (ret < 0) {
		PX4_WARN("Failed to schedule first cycle");
		return ret;
	}

	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

void TFMini::cycle()
{
	// PX4_INFO("cycle");
	int num_bytes_read = 0;
	uint8_t rx_buffer[BUFFER_SIZE];
	memset(rx_buffer, 0, sizeof(rx_buffer));

#if defined(__PX4_QURT)
	/* For QURT, just use read for now, since this doesn't block, the read is
	   delayed by a bit longer than it would take to transmit a full message
		 given the protocol's baud rate. */
	// usleep(TFMINI_WAIT_BEFORE_READ_MICRO_SECS);
	num_bytes_read = ::read(_uart_file_des, rx_buffer, sizeof(rx_buffer));
	memcpy(_buffer, rx_buffer, num_bytes_read);

#else
	// Wait for data to be received, using sensor rate as timeout in order to have
	// the task still appear responsive
	struct pollfd fds;
	fds.fd = _uart_file_des;
	fds.events = POLLIN;
	fds.revents = 0;
	// PX4_INFO("polling");
	int ret = ::poll(&fds, 1, 1000.0 / TFMINI_SENSOR_RATE);

	if (ret > 0 && fds.revents & POLLIN) {
		/*
		 * We are here because poll says there is some data, so this
		 * won't block even on a blocking device. But don't read immediately
		 * by 1-2 bytes, wait for some more data to save expensive read() calls.
		 * If we have all requested data available, read it without waiting.
		 * If more bytes are available, we'll go back to poll() again.
		 */
#ifdef __PX4_NUTTX
		int err = 0;
		int bytesAvailable = 0;
		err = ::ioctl(_uart_file_des, FIONREAD, (unsigned long)&bytesAvailable);

		PX4_INFO("bytesAvailable: %u", bytesAvailable);

		if ((err != 0) || (bytesAvailable < TFMINI_FRAME_SIZE)) {
			usleep(5 * TFMINI_WAIT_BEFORE_READ_MICRO_SECS);
		}

#else
		PX4_INFO("reading");
		usleep(TFMINI_WAIT_BEFORE_READ_MICRO_SECS);
#endif
		num_bytes_read = ::read(_uart_file_des, rx_buffer, sizeof(rx_buffer));
		PX4_INFO("num_bytes_read: %u", num_bytes_read);
		memcpy(_buffer, rx_buffer, num_bytes_read);
	}

#endif


	// Parse and publish sensor reading
	if (num_bytes_read > 0) {
		bool parsed = TFMiniProto::parse(rx_buffer, num_bytes_read, &sensor_msg);

		/* publish sensor data if this is the primary */
		if (parsed && _distance_sensor_topic != nullptr) {
			orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &sensor_msg);
		}
	}
}

void TFMini::cycle_trampoline(void *arg)
{
	PX4_INFO("cycle_trampoline");
	const int cycle_start_time = hrt_absolute_time();
	TFMini *tfmini = reinterpret_cast<TFMini *>(arg);
	tfmini->cycle();

	// Schedule next execution
	if (!tfmini->should_exit()) {
		// NOTE: The actual scheduling frequency is still not correct (too slow)
		work_queue(HPWORK, &_work, (worker_t)&TFMini::cycle_trampoline, tfmini,
			   USEC2TICK(int(1000000 / TFMINI_SENSOR_RATE - (hrt_absolute_time() - cycle_start_time))));

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
	PX4_INFO("running on device %s", _device_path);
	return PX4_OK;
}

int TFMini::init()
{
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
	int uart_fd = ::open(device, O_RDONLY | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	// int uart_fd = px4_open(device, O_RDWR | O_NOCTTY);
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
	struct termios config;

	if(!isatty(uart_fd)) {
		PX4_ERR("TFMiniProto: Not a TTY device");
		return PX4_ERROR;
	}

	/* Make sure the struct does not have anything in it*/
	// memset(&config, 0, sizeof(struct termios));

	// NOTE: For some reason the flag configuration messes everything up
	//
	/* Get the current configurations of the termios device. */
	if (tcgetattr(uart_fd, &config) < 0)
	{
		PX4_ERR("TFMiniProto: tcgetattr call failed.");
		return PX4_ERROR;
	}

	///////
	// config.c_oflag &= ~ONLCR;
	// config.c_cflag &= ~CRTSCTS;
	///////

	// Setting 8N1 character size (8), parity (None) and stop bits (1)
	config.c_cflag &= ~PARENB;
	config.c_cflag &= ~CSTOPB;
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;


	// No line processing
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Disable flow control
	// config.c_cflag &= ~CNEW_RTSCTS;  // NOTE: Does not compile

	// Raw input
	config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Input flags - Turn off input processing
	config.c_iflag &= ~(IUCLC | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXOFF | IXON);
	// config.c_iflag |= (IGNBRK | IGNPAR);
	// config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags
	config.c_oflag = 0;

	// Set baud rate
	PX4_INFO("Setting baud rate");
	if(cfsetispeed(&config, TFMINI_BAUD_RATE) < 0 ||
		 cfsetospeed(&config, TFMINI_BAUD_RATE) < 0) {
		PX4_ERR("TFMiniProto: failed to set baudrate\n");
		return PX4_ERROR;
	}

	// Apply the config changes
	if(tcsetattr(uart_fd, TCSANOW, &config) < 0)
	{
		PX4_ERR("TFMiniProto: failed to apply termios configurationd\n");
		return PX4_ERROR;
	}



	PX4_INFO("Done with UART setup!");
	return PX4_OK;
}

bool TFMiniProto::parse(uint8_t *const buffer, size_t buff_len, distance_sensor_s * const msg) {
	// Print buffer
	printf("buffer: ");
	for (size_t i = 0; i < buff_len; i++){
		printf("%X ", buffer[i]);
	}
	printf("\n");

	// Look for message header
	// PX4_INFO("buffer: %s", buffer);
	size_t header_index;
	for (header_index = 0; header_index < buff_len-TFMINI_FRAME_SIZE; header_index++){
		if(buffer[header_index] == TFMINI_FRAME_HEADER &&
			 buffer[header_index+1] == TFMINI_FRAME_HEADER){
			PX4_INFO("Header found");
			break;  // header found
		}else if(header_index+1==buff_len-TFMINI_FRAME_SIZE){
			// Last iteration and no header was found
			PX4_INFO("No header found");
			return false;
		}
	}

	// make sure enough bytes have been received to make up entire frame
	if (buff_len < header_index+TFMINI_FRAME_SIZE){
		PX4_WARN("Incomplete message received");
		return false;
	}

	// Compute and compare checksum
	// Checksum is the lower 8 bit of the sum of the frame's payload, without crc:
	// byte1 + byte2 + byte3 + .. + byte8
	uint16_t expected_checksum = 0;
	for (size_t byte = 0; byte < TFMINI_BYTE_POS_CRC; byte++){
		expected_checksum += buffer[byte];
	}
	if (buffer[TFMINI_BYTE_POS_CRC]!=expected_checksum%256){
		PX4_WARN("crc not matching, dropping measurement");
		return false; // CRC mismatch, drop message
	}

	msg->timestamp = hrt_absolute_time();
	msg->current_distance = (buffer[header_index+TFMINI_BYTE_POS_DIST_L] +
													(buffer[header_index+TFMINI_BYTE_POS_DIST_H] << 8))/100.0;
	msg->covariance = 0.0f;  // TODO: Can this be a static value from the data sheet?

	PX4_INFO("distance: %f", (double)msg->current_distance);
	return true;
}

extern "C" __EXPORT int tfmini_main(int argc, char *argv[]);

int tfmini_main(int argc, char *argv[])
{
	// return TFMini::main(argc, argv);

	// PX4_INFO("task_spawn");
	// TFMini *tfmini = TFMini::instantiate(argc, argv);
	//
	// if (tfmini == nullptr) {
	// 	PX4_ERR("Failed to instantiate module");
	// 	return PX4_ERROR;
	// }
	//
	// for (size_t i=0; i < 10; i++)
	// {
	// 	PX4_INFO("Calling cycle");
	// 	tfmini->cycle();
	// 	PX4_INFO("");
	// 	usleep(TFMINI_WAIT_BEFORE_READ_MICRO_SECS);
	// }
	//
	// delete tfmini;
	//
	//
	// return PX4_OK;


	PX4_INFO("Opening UART");
	int fd = px4_open("/dev/ttyS2", O_RDWR | O_NOCTTY);
	if(fd<0) {
		PX4_ERR("Could not open serial device");
		return PX4_ERROR;
	}

	/// UART CONFIG ///
	struct termios config;

	if (!isatty(fd)) {
		PX4_ERR("Not a tty device");
		return PX4_ERROR;
	}

	if(tcgetattr(fd, &config) < 0){
		PX4_ERR("Could not get termios config");
		return PX4_ERROR;
	}
	// Raw input
	config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

	// No line processing
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	config.c_oflag &= ~OPOST;

	// 8N1
	config.c_cflag |= (CLOCAL | CREAD);
	config.c_cflag &= ~PARENB;
	config.c_cflag &= ~CSTOPB;
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;
	// config.c_cflag=CS8|CREAD|CLOCAL;

	if(cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
     PX4_ERR("Could not set baud rate");
		 return PX4_ERROR;
	}

	if(tcsetattr(fd, TCSANOW, &config) < 0) {
		PX4_ERR("Could not submit config");
		return PX4_ERROR;
	}

	// usleep(30000);
	unsigned char buffer[255];  /* Input buffer */
	// unsigned char collective_buffer[2550];
  unsigned int  nbytes;       /* Number of bytes read */
	unsigned int  nbytes_tot = 0;
	// struct pollfd fds;
	// fds.fd = fd;
	// fds.events = POLLIN;
	// fds.revents = 0;

	usleep(10000);
	for (size_t i = 0; i < 100; i++){
		// PX4_INFO("polling");
		// int ret = ::poll(&fds, 1, 1000.0 / TFMINI_SENSOR_RATE);

		memset(buffer, 0, sizeof(buffer));

		// if (ret > 0 && fds.revents & POLLIN) {
		nbytes = read(fd, &buffer[0], 255);
		nbytes_tot+=nbytes;
		PX4_INFO("Received %u bytes", nbytes);
		if (nbytes>0) {
			printf("buffer: ");
			for (size_t byte = 0; byte<nbytes; byte++) {
				printf("%X ", buffer[byte]);
			}
			printf("\n\n");
			}
		// }
		usleep(1000);
	}

	PX4_INFO("Closing UART");
	px4_close(fd);
	return PX4_OK;
}

} // namespace tfmini

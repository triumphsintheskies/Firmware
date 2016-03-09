/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the GPS on a serial port
 */

#ifndef __PX4_QURT
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <fcntl.h>
#endif

#include <systemlib/scheduling_priorities.h>
#include <px4_tasks.h>
#include <px4_defines.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#define GPS_DEFAULT_UART_PORT "/dev/tty-4"


class GPS
{
public:
	GPS();
	~GPS();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */

	int init();

private:

	//int				_serial_fd;					///< serial interface to GPS
	volatile int			_task;						///< worker task



	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);

namespace
{

GPS *g_dev = nullptr;

}


GPS::GPS()
{
}

GPS::~GPS()
{
}

int
GPS::init()
{

	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1200, (px4_main_t)&GPS::task_main_trampoline, nullptr);

	if (_task < 0) {
		PX4_WARN("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
GPS::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
GPS::task_main()
{
	/* open the serial port */

	//_serial_fd = ::open("/dev/tty-4", O_RDWR);

	//if (_serial_fd < 0) {
	//	while (true) {
	//		PX4_WARN("failed to open serial port: %s err: %d", _port, errno);
	//	}

	//	/* tell the dtor that we are exiting, set error code */
	//	_task = -1;
	//	return;
	//}

	//PX4_WARN("exiting");

	//::close(_serial_fd);

	/* tell the dtor that we are exiting */
		while (true) {
			PX4_WARN("looping");
			usleep(1000000);
		}
	_task = -1;



	//px4_task_exit(0);
}




/**
 * Local functions in support of the shell command.
 */
namespace gps
{

GPS	*g_dev = nullptr;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new GPS();

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	return;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("start failed");
	return;
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	PX4_ERR("GPS reset not supported");
	return;
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "not running");
	}

	return;
}

} // namespace


int
gps_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		gps::start();
	}

	if (!strcmp(argv[1], "stop")) {
		gps::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		gps::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		gps::reset();
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		gps::info();
	}

	return 0;
}

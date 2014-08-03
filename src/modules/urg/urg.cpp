/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: @author Antonio Sanniravong <antonio.sanniravong@polymtl.ca>
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
 * @file urg.c
 * Hokuyo URG-04LX device driver.
 */
 
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <fcntl.h>
#include <semaphore.h>

#include <sys/types.h>
#include <sys/prctl.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/parameter_update.h>

#include <mavlink/mavlink_log.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <uORB/topics/subsystem_info.h>

#include "urg_params.h"
#include "scip.h"

#define URG_CONVERSION_INTERVAL	83334
#define URG_TAKE_RANGE_REG		'd'
#define URG_MIN_DISTANCE		0.0f
#define URG_MAX_DISTANCE		40.0f
#define URG_DEFAULT_PORT		"/dev/ttyS6"

extern "C" __EXPORT int urg_main(int argc, char *argv[]);

int urg_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int urg_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: urg {start|stop|status} [-v]\n\n");
	exit(1);
}

int urg_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		verbose_mode = false;

		if (argc > 1)
			if (!strcmp(argv[2], "-v")) {
				verbose_mode = true;
			}

		thread_should_exit = false;
		urg_task = task_spawn_cmd("urg",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 50, 5000,
					       urg_thread_main,
					       (argv) ? (const char **) &argv[2] : (const char **) NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("app not started");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("app is running");

		} else {
			warnx("app not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int urg_thread_main(int argc, char *argv[]) {

	thread_running = true;

	/* Initialize structs */
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));

	/* Subscribe to uORB topics */
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Advertise on actuators topic */
	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);

	/* Initialize parameter handles */
	struct urg_params params;
	struct urg_param_handles urg_param_handles;
	parameters_init(&urg_param_handles);

	/* First parameter read */
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update);

	/* First parameter update */
	parameters_update(&urg_param_handles, &params);

	/* Initialize MAVLink fd for output to QGC */
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[LED] started");

	/* Polling fds */
	struct pollfd fds[1];
	fds[0].fd = local_pos_sub;
	fds[0].events = POLLIN;

	/* Variable initializations */
	int error_counter = 0;
	hrt_abstime t_prev = 0;		// Absolute time of previous iteration of main loop

	/*
	* URG UART port initializations
	*/

	/* Open fd */

	int _fd = ::open(URG_DEFAULT_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
	warnx("FAIL: laser fd");
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
	uart_config.c_cflag |= CS8;

	unsigned speed = B19200;

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
	 warnx("ERR CFG: %d ISPD", termios_state);
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
	 warnx("ERR CFG: %d OSPD\n", termios_state);
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
	 warnx("ERR baud %d ATTR", termios_state);
	}

	usleep(1000);

	/* Switch to SCIP2.0 */
	::write(_fd, "SCIP2.0\n", 8);
	usleep(2000);

	char buf[10];
	memset(buf, 0, 10);
	int lfcount = 0;
	int n = 0;

	/* Read answer to clear the "SCIP2.0 LF Status LF LF" response from UART buffer */
	while(lfcount < 3) {
		n = read(_fd, &buf, sizeof(buf));
		for (int i = 0; i < n; i++) {
			//printf("%c", buf[i]);
			if (buf[i] == '\n') {
				lfcount++;
			}
		}
	}

	while (!thread_should_exit) {

		/* Read UART buffer & parse messages */
		// TODO: Andre's magic

		/* Wait for update for 1000 ms */
		int poll_result = poll(fds, 1, 1000);
		hrt_abstime t = hrt_absolute_time();

	 	/* Calculate time difference since last iteration of loop */
	 	float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
	 	dt = fmaxf(fminf(0.05, dt), 0.005);		// Constrain dt from 5 to 50 ms
	 	t_prev = t;

	 	if (poll_result == 0) {
	 		/* No new data */
	 		if (verbose_mode) {
	 			printf("[urg] Got no data within a second. \n");
	 		}
	 	} else if (poll_result < 0) {
	 		/* ERROR */
	 		if (error_counter < 10 || error_counter % 50 == 0) {
	 			/* Use error counter to prevent flooding */
	 			if (verbose_mode) {
	 				printf("[urg] ERROR return value from poll(): %d\n", poll_result);
	 			}
	 		}
	 		error_counter++;

	 	} else {

	 		/* Parameter update */
	 		bool updated;
	 		orb_check(parameter_update_sub, &updated);
	 		if (updated) {
	 			struct parameter_update_s update;
	 			orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
	 			parameters_update(&urg_param_handles, &params);
	 		}

	 		/* Poll data */
	 		if (fds[0].revents & POLLIN) {

	 			/* Copy data to local buffer */
	 			orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

	 			/* Do stuff */

	 		}
	 	}

	 	/* Copy controls and whatever to struct */
	 	//local_pos_sp.x = something;

	 	/* Publish output */
	 	orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);

	 }

	 ::close(_fd);
	 warnx("stopped");
	 thread_running = false;
	 return 0;
}

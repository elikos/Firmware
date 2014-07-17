/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file flow_led.c
 * PX4Flow assistive LED lighting control.
 */
 
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
 #include <uORB/topics/parameter_update.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "flow_led_params.h"

__EXPORT int flow_led_main(int argc, char *argv[]);

int flow_led_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int flow_led_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: flow_led {start|stop|status} [-v]\n\n");
	exit(1);
}

int flow_led_main(int argc, char *argv[])
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
		flow_led_task = task_spawn_cmd("flow_led",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 50, 5000,
					       flow_led_thread_main,
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

int flow_led_thread_main(int argc, char *argv[]) {

	thread_running = true;

	/* Initialize structs */
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* Subscribe to uORB topics */
	int flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Advertise on actuators topic */
	orb_advert_t actuators_pub = orb_advertise(ORB_ID(actuator_controls_1), &actuators);

	/* Initialize parameter handles */
	struct flow_led_params params;
	struct flow_led_param_handles flow_led_param_handles;
	parameters_init(&flow_led_param_handles);

	/* First parameter read */
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update);

	/* First parameter update */
	parameters_update(&flow_led_param_handles, &params);

	/* Initialize MAVLink fd for output to QGC */
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[LED] started");

	/* Polling fds */
	struct pollfd fds[] = {
		{ .fd = flow_sub,   .events = POLLIN },
		// Poll manual sp for debugging purposes
		{ .fd = manual_sub,	.events = POLLIN }
	};

	/* Variable initializations */
	int error_counter = 0;
	int loop_counter = 0;
	int flow_q = 0;				// Flow quality factor
	int flow_q_err = 0;			// Flow quality error from setpoint parameter
	float sonar = 0.0f;			// Sonar height measurement
	float sonar_avg = 0.0f;		// Sonar average height measurement
	float led_out = 0.0f;		// LED output intensity (0.0 - 1.0)
	float led_int = 0.0f;		// LED output integral
	float led_err = 0.0f;		// LED error calculated from flow Q error
	float led_err_prev = 0.0f;	// LED previous error
	float throttle = 0.0f;		// Current throttle stick position
	hrt_abstime t_prev = 0;		// Absolute time of previous iteration of main loop

	while (!thread_should_exit) {

		/* Wait for update for 1000 ms */
		int poll_result = poll(fds, 2, 1000);
		hrt_abstime t = hrt_absolute_time();

		/* Calculate time difference since last iteration of loop */
		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.05, dt), 0.005);		// Constrain dt from 5 to 50 ms
		t_prev = t;
 
		if (poll_result == 0) {
			/* No new flow data */
			if (verbose_mode) {
				printf("[flow_led] Got no data within a second. \n");
			}
		} else if (poll_result < 0) {
			/* ERROR */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* Use error counter to prevent flooding */
				if (verbose_mode) {
					printf("[flow_led] ERROR return value from poll(): %d\n", poll_result);
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
				parameters_update(&flow_led_param_handles, &params);
			}
 
			/* Poll flow data */
			if (fds[0].revents & POLLIN) {

				/* Copy flow data to local buffer */
				orb_copy(ORB_ID(optical_flow), flow_sub, &flow);
				flow_q = flow.quality;
				sonar = flow.ground_distance_m;

				/* Update flow quality error using setpoint parameter */
				flow_q_err = params.flow_q_sp - flow_q ;
				led_err = flow_q_err / 255.0f;

				// TODO: Sonar average calculation

				/* LED PID control */
				led_int += led_err * params.led_i * dt;
				led_out += led_err * params.led_p + (led_err_prev - led_err) * params.led_d / dt + led_int;
				led_err_prev = led_err;

			}

			/*
			// Poll manual setpoint for debugging purposes
			if (fds[1].revents & POLLIN) {

				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
				throttle = manual.z;

			}
			*/
		}

		/*
		// TEMP: Blink LED
		if (led_out < 0.5f && (loop_counter % 2 == 0)){
			led_out = 1.0f;
			mavlink_log_info(mavlink_fd, "Led output: %.2f", led_out)
		} else if (led_out > 0.5f && (loop_counter % 2 == 0)) {
			led_out = 0.0f;
			mavlink_log_info(mavlink_fd, "Led output: %.2f", led_out)
		}
		*/

		/*
		// TEMP: Assign throttle stick to LED output
		led_out = throttle;
		*/

		/* Limit LED output */
		if (led_out > 1.0f){
			led_out = 1.0f;
		} else if (led_out < 0.0f) {
			led_out = 0.0f;
		}

		/* Copy controls and timestamp to struct */
		actuators.control[0] = led_out;
		actuators.timestamp = t;

		/* Publish PWM output */
		orb_publish(ORB_ID(actuator_controls_1), actuators_pub, &actuators);

		loop_counter++;

	}
	warnx("stopped");
	thread_running = false;
	return 0;
}

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
#include <uORB/topics/obstacle_detection.h>

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
#include "urg04lx.h"

#define URG_CONVERSION_INTERVAL	83334
#define URG_TAKE_RANGE_REG		'd'
#define URG_MIN_DISTANCE		0.0f
#define URG_MAX_DISTANCE		40.0f
#define URG_DEFAULT_PORT		"/dev/ttyS6"
#define URG_NB_OF_CLUSTERS		15

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
	struct obstacle_detection_s obstacle_detection;
	memset(&obstacle_detection, 0, sizeof(obstacle_detection));

	/* Subscribe to uORB topics */
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Advertise on actuators topic */
	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t obstacle_detection_pub = orb_advertise(ORB_ID(obstacle_detection), &obstacle_detection);

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
	int* distances;
	int distances_avg[URG_NB_OF_CLUSTERS];
	bool distances_avg_inited = false;
	bool sp_reached[2] = {false, false};
	int sp_number = 0;
	bool lidar_updated = false;
	hrt_abstime lidar_update_t;
	int min_angle= 0;				// this parameter and following used to implement sideway dodge
	int min_obstacle_detection = 2000;
	int DANGER_ZONE = 1500;
	float PI = 3.14159265359;
	/*angle table initialisation based on URG_NB_OF_CLUSTERS */
			
	int urg_angle_table[URG_NB_OF_CLUSTERS];
	for (int i = 0; i< URG_NB_OF_CLUSTERS; i++){
		urg_angle_table[i] = 91 - (180/URG_NB_OF_CLUSTERS)*i
	}

	/* URG UART port initializations
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

	//Create URG object
	URG04LX laser(_fd);

	while (!thread_should_exit) {


		/* Calculate time difference since last iteration of loop */
		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		//dt = fmaxf(fminf(0.05, dt), 0.005);		// Constrain dt from 5 to 50 ms
		t_prev = t;

		/* Read UART buffer & parse messages */
		if (t > lidar_update_t + 200000.0f) {
			lidar_update_t = t;
			lidar_updated = true;
			distances = laser.getRangeResponse();
		}

		if (!distances_avg_inited && lidar_updated) {
			for (int i = 0; i < URG_NB_OF_CLUSTERS; i++) {
			distances_avg[i] = distances[i];
			}
			distances_avg_inited = true;
		}

		if (lidar_updated) {
			for (int i = 0; i < URG_NB_OF_CLUSTERS; i++) {
				/* If distance is greater than 20mm, take the value in count to measure average. */
				if (distances[i] > 20) {
					distances_avg[i] += (distances[i] - distances_avg[i]) * 0.5;
				}
				/* If distance is lower than 10mm, lidar is reading infinity. Adjust avg towards 5000mm. */
				else {
					distances_avg[i] += (5000 - distances_avg[i]) * 0.5;
				}
			}
			lidar_updated = false;
		}

		/* Wait for update for 1000 ms */
		int poll_result = poll(fds, 1, 1000);

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

	 		bool updated;

	 		/* Parameter update */
	 		orb_check(parameter_update_sub, &updated);
	 		if (updated) {
	 			struct parameter_update_s update;
	 			orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
	 			parameters_update(&urg_param_handles, &params);
	 		}

	 		/* Update position setpoints */
	 		orb_check(local_pos_sp_sub, &updated);
			if (updated) {
				if (!obstacle_detection.obstacle_detected) {
					/* Keep latest setpoint before switching to obstacle avoidance mode */
					orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
				}
			}
			

			
	 		/* Poll data */
	 		if (fds[0].revents & POLLIN) {

	 			/* Copy data to local buffer */

	 			orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

	 			/* Find the minimum (closest) distance from the distances array */
	 			float min_avg_dist = distances_avg[0]/1000;

	 			for (int i = 0; i < URG_NB_OF_CLUSTERS; i++) {
	 				if (distances_avg[i] < min_avg_dist) {
	 					min_avg_dist = distances_avg[i]/1000;
	 					min_angle = i;
	 				}
	 			}
			
	 			/* Activate obstacle detection mode if closest obstacle is close enough */
				 if (min_avg_dist < min_obstacle_detection) { //TODO: make this a param
	 				obstacle_detection.obstacle_detected = true;
	 				obstacle_detection.timestamp = hrt_absolute_time();
	 				/* symboles magiques : a updater */
	 				float F = d*sin(min_angle*2*PI/360);
	 				float deltaX = d*sin(min_angle*2*PI/360);
	 				float deltaY = DANGER_ZONE - abs(F);

					// application d'une rotation de Yaw
					if (deltaY > 0){
					float deltaX_yawed = deltaX*cos(YAW) - deltaY*sin(YAW);
					float deltaY_yawed = deltaX*sin(YAW) + deltaY*cos(YAW);
					
					// determination des nouvelles coordonnes en x,y
					obstacle_detection.x_sp = local_pos.x + deltaX_yawed;
					obstacle_detection.y_sp = local_pos.y + deltaY_yawed;
					obstacle_detection.z_sp = local_pos.z;
					}
	 				}
					
	 			/* Deactivate obstacle detection mode if no threat is detected within timeout time after reaching new setpoint */
	 			else if (obstacle_detection.timestamp + 2000 < hrt_absolute_time()) { //TODO: make timeout a parameter
	 				obstacle_detection.obstacle_detected = false;
					}
	 			}

				
	 	/*		if (sp_reached[0] == 0) {
	 				/* First setpoint */
	 				/* Set climbing setpoint (climb to 2.5m) at current position */
	 	/*		p_number = 1;
					obstacle_detection.x_sp = local_pos.x;
					obstacle_detection.y_sp = local_pos.y;
					obstacle_detection.z_sp = -2.5f;
	 			} else if (sp_reached[1] == 0) {
	 				/* Second setpoint */
	 				/* Fly to original setpoint at 2.5m altitude */
	 	/*		p_number = 2;
	 				obstacle_detection.x_sp = local_pos_sp.x;
					obstacle_detection.y_sp = local_pos_sp.y;
					obstacle_detection.z_sp = -2.5f;
	 			}
		*/
	 			//TODO: switch to XY avoidance setpoint

	 			orb_publish(ORB_ID(obstacle_detection), obstacle_detection_pub, &obstacle_detection);

	 		}
	 	}

	 	/* Copy controls and whatever to struct */
	 	//local_pos_sp.x = something;

	 	/* Publish output */
	 	//orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);

	 ::close(_fd);
	 warnx("stopped");
	 thread_running = false;
	 return 0;
}

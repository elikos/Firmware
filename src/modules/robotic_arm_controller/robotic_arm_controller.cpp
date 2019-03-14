/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include "robotic_arm_controller.h"

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <px4_log.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>

#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>

//from template
#include <px4_getopt.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

int RoboticArmController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Section that describes the provided module functionality.
		This is a template for a module running as a task in the background with start/stop/status functionality.
		### Implementation
		Section describing the high-level implementation of this module.
		### Examples
		CLI usage example:
		$ module start -f -p 42
		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int RoboticArmController::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int RoboticArmController::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}
	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int RoboticArmController::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

RoboticArmController *RoboticArmController::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	RoboticArmController *instance = new RoboticArmController(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

RoboticArmController::RoboticArmController(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}


void Module::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


void RoboticArmController::run() //code d'arnaud 
{
	PX4_INFO("Hello Sky!");
    int command_sub = orb_subscribe(ORB_ID(vehicle_command));

    actuator_controls_s actuators = {};
    orb_advert_t _actuator_pub = nullptr;

    px4_pollfd_struct_t fds[1];
    fds[0].fd = command_sub;
    fds[0].events = POLLIN;

	while (!should_exit()) {
        
        // wait for up to 1000ms for data
        int poll_ret = px4_poll(fds, 1, 1000);
        if (poll_ret < 0) {
            PX4_ERR("FATAL, px4_poll returned %d", poll_ret);
            break;

        } else if(poll_ret == 0) {
            // Timeout: let the loop run anyway, don't do `continue` here <- default command

            continue;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct vehicle_command_s cmd;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_command), command_sub, &cmd);
                if (cmd.command == NAV_CMD_DO_SET_SERVO) {
                    PX4_INFO("Setting servo #%d at %8.4f us", (int)cmd.param1, (double)cmd.param2);

                    actuators.timestamp = hrt_absolute_time();

                    // params[0] actuator number to be set 0..5 (corresponds to AUX outputs 1..6)
                    // params[1] new value for selected actuator in ms 900...2000
                    actuators.control[(int)cmd.param1] = -1.0f / 2000 * cmd.param2;

                    if (_actuator_pub != nullptr) {
                        orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &actuators);

                    } else {
                        _actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuators);
                    }
                }
            }
        }
    }
}

int module_main(int argc, char *argv[])
{
	return RoboticArmController::main(argc, argv);
}


// Old code under (arnaud)

/*int robotic_arm_controller_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");
    int command_sub = orb_subscribe(ORB_ID(vehicle_command));

    actuator_controls_s actuators = {};
    orb_advert_t _actuator_pub = nullptr;

    px4_pollfd_struct_t fds[1];
    fds[0].fd = command_sub;
    fds[0].events = POLLIN;

    while (true) {
        int poll_ret = px4_poll(fds, 1, 1000);
        if (poll_ret < 0) {
            PX4_ERR("FATAL, px4_poll returned %d", poll_ret);
            break;
        } else if(poll_ret == 0) {
            continue;
        } else {
            if (fds[0].revents & POLLIN) {
                //obtained data for the first file descriptor 
                struct vehicle_command_s cmd;
                // copy sensors raw data into local buffer 
                orb_copy(ORB_ID(vehicle_command), command_sub, &cmd);
                if (cmd.command == NAV_CMD_DO_SET_SERVO) {
                    PX4_INFO("Setting servo #%d at %8.4f us", (int)cmd.param1, (double)cmd.param2);

                    actuators.timestamp = hrt_absolute_time();

                    // params[0] actuator number to be set 0..5 (corresponds to AUX outputs 1..6)
                    // params[1] new value for selected actuator in ms 900...2000
                    actuators.control[(int)cmd.param1] = -1.0f / 2000 * cmd.param2;

                    if (_actuator_pub != nullptr) {
                        orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &actuators);

                    } else {
                        _actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuators);
                    }
                }
            }
        }
    }

    return OK;
}
*/

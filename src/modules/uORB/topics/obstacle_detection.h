/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file obstacle_detection.h
 * Definition of the obstacle_detection uORB topic.
 *
 * Obstacle detection in position controller should be noticed through this topic.
 *
 * @author Antonio Sanniravong <antonio.sanniravong@polymtl.ca>
 */

#ifndef OBSTACLE_DETECTION
#define OBSTACLE_DETECTION

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"
#include "vehicle_status.h"

/**
 * @addtogroup topics @{
 */


/**
 * Obstacle detection sequence
 *
 * Enable the obstacle detection sequence in position controller
 */

struct obstacle_detection_s {
	uint64_t timestamp; /**< in microseconds since system start, is set whenever the writing thread stores new data */
	bool obstacle_detected;		/* True if lidar detects nearby object */

	float x_sp;
	float y_sp;
	float z_sp;

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(obstacle_detection);

#endif

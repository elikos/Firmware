/****************************************************************************
 *
 *   Copyright (c) 2015 Elikos Team. All rights reserved.
 *   Authors: @author Antoine Mignon <mignon.antoine@gmail.com>
 *			  @author Alexandre Borowczyk <borowczyk.alexandre@gmail.com>
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
 * @file nlibs_control_params.c
 * Multicopter non linear integral backstepping controller parameters.
 *
 * @author Antoine Mignon <mignon.antoine@gmail.com>
 * @author Alexandre Borowczyk <borowczyk.alexandre@gmail.com>
 */

#include <systemlib/param/param.h>

/*
 * Quadrotor mass (Kg)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QMASS, 1.0f);

/*
 * Equivalent inertial moment around x-axis (Nm.s²/rad)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QIX_MOMENT, 0.0001f);

/*
 * Equivalent inertial moment around y-axis (Nm.s²/rad)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QIY_MOMENT, 0.0001f);

/*
 * Equivalent inertial moment around z-axis (Nm.s²/rad)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QIZ_MOMENT, 0.0001f);

/*
 * Arm length (m)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QARM_LENGTH, 0.01f);

 /*
  * Rotor drag coefficient
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QDRAG_COEFF, 0.01f);

/*
 * Linear x-axis drag (N.s/m)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QXLIN_DRAG, 0.01f);

 /*
  * Linear y-axis drag (N.s/m)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QYLIN_DRAG, 0.01f);

 /*
  * Linear z-axis drag (N.s/m)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QZLIN_DRAG, 0.01f);

 /*
  * Rotational x-axis drag (N.s/rad)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QXROT_DRAG, 0.01f);

 /*
  * Rotational y-axis drag (N.s/rad)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QYROT_DRAG, 0.01f);

 /*
  * Rotational z-axis drag (N.s/rad)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QZROT_DRAG, 0.01f);

/*
 * Rotor radius (m)
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_QROTOR_RADIUS, 0.25f);

 /*
  * Rotor twist (??)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QROTOR_TWIST_ANGLE, 0.01f);

 /*
  * Root angle ?? (??)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QROTOR_ROOT_ANGLE, 0.01f);

 /*
  * Motor constant (??)
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
 PARAM_DEFINE_FLOAT(NLIBSC_QMOTOR_CST, 0.2f);

/* 
 * Minimum vertial thrust
 * It is recommanded to set the minimal vertical thrust to a value greater than 0 to avoid free fall
 * @min 0.0
 * @max 1.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_THR_MIN, 0.1f);

/* 
 * Maximal vertial thrust
 * Limit the maximum allowed thrust
 * @min 0.0
 * @max 1.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_THR_MAX, 1.0f);

 /*
  * X-axis positition gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_X_GAIN, 1.0f);

 /*
  * Y-axis positition gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_Y_GAIN, 1.0f);

 /*
  * X-axis speed gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_X_VEL_GAIN, 1.0f);

 /*
  * Y-axis speed gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_Y_VEL_GAIN, 1.0f);

 /*
  * Roll angle gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_PHI_GAIN, 1.0f);  

 /*
  * Pitch angle gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_THETA_GAIN, 1.0f);  

 /*
  * Roll rate gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_PHI_RATE_GAIN, 1.0f);  

 /*
  * Pitch rate gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_THETA_RATE_GAIN, 1.0f);  

 /*
  * Yaw angle gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_PSI_GAIN, 1.0f); 

 /*
  * Z-axis position gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_Z_GAIN, 1.0f);

 /*
  * Yaw rate gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_PSI_RATE_GAIN, 1.0f);

 /*
  * Z-axis speed gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_Z_VEL_GAIN, 1.0f);

 /*
  * Motor 1 input gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_F1_GAIN, 1.0f);

 /*
  * Motor 2 input gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_F2_GAIN, 1.0f);

 /*
  * Motor 3 input gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_F3_GAIN, 1.0f);

 /*
  * Motor 4 input gain. Must be positive
  * @min 0.0
  * @group Multicopter NLIBS Control
  */
  PARAM_DEFINE_FLOAT(NLIBSC_F4_GAIN, 1.0f);

/* TO REVIEW */

/**
 * Maximum roll rate
 * Limit for large roll rotations (avoid large control output and mixer saturation).
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_ROLL_RATE_MAX, 360.0f);

/**
 * Maximum pitch rate
 * Limit for large pitch rotations (avoid large control output and mixer saturation).
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_PITCH_RATE_MAX, 360.0f);

/**
 * Maximum yaw rate
 * Limit for large yaw rotations (avoid large control output and mixer saturation).
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter NLIBS Control
 */
 PARAM_DEFINE_FLOAT(NLIBSC_YAW_RATE_MAX, 360.0f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_XY_VEL_MAX, 5.0f);

/**
 * Horizontal velocity feed forward
 *
 * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_XY_FF, 0.5f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_TILTMAX_LND, 15.0f);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_LAND_SPEED, 1.0f);

/**
 * Max manual roll
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_MAN_R_MAX, 35.0f);

/**
 * Max manual pitch
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_MAN_P_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @group Multicopter NLIBS Control
 */
PARAM_DEFINE_FLOAT(NLIBSC_MAN_Y_MAX, 120.0f);


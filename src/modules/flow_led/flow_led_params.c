/****************************************************************************
 *
 *   Copyright (C) 2014 Antonio Sanniravong. All rights reserved.
 *   Author: 	Antonio Sanniravong	<antonio.sanniravong@polymtl.ca>
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

/*
 * @file flow_led_params.c
 *
 * Parameters for PX4Flow assistive LED controller
 */

#include "flow_led_params.h"

PARAM_DEFINE_FLOAT(LED_P, 0.5f);
PARAM_DEFINE_FLOAT(LED_I, 0.0f);
PARAM_DEFINE_FLOAT(LED_D, 0.0f);
PARAM_DEFINE_INT32(LED_FLOW_Q_SP, 240);

int parameters_init(struct flow_led_param_handles *h)
{
	h->led_p = param_find("LED_P");
	h->led_i = param_find("LED_I");
	h->led_d = param_find("LED_D");
	h->flow_q_sp = param_find("LED_FLOW_Q_SP");

	return OK;
}

int parameters_update(const struct flow_led_param_handles *h, struct flow_led_params *p)
{
	param_get(h->led_p, &(p->led_p));
	param_get(h->led_i, &(p->led_i));
	param_get(h->led_d, &(p->led_d));
	param_get(h->flow_q_sp, &(p->flow_q_sp));

	return OK;
}

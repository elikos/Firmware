/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file urg04lx.h
 * Hokuyo URG-04LX device driver.
 * @author Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
 * @remarks Based off the official Hokuyo QRK_URG_DRIVER
 */

#pragma once

class URG04LX {
public:
	typedef enum {
		URG_COMMUNICATION_3_BYTE,
		URG_COMMUNICATION_2_BYTE,
	} urg_range_data_byte_t;

	/**
	 * @brief URG04LX constructor
	 * 
	 * @param fd An opened file descriptor to the LIDAR
	 */
	URG04LX(int fd);
	virtual ~URG04LX(void);

	bool getVersion();

	/**
	 * @brief Scan a step range
	 * 
	 * @param comRange 2 or 3 byte encoding
	 * @param startStep Starting step minimum 10
	 * @param endStep Last step maximum 750
	 * @param clusterCount Between 0 and 99, a cluster allows us to get the
	 * 		closest measurement within a defined cluster. e.g. if you cluster
	 * 		3 points 233 256 156, URG will return 156.
	 * @param scanInterval If you are requesting multiple scans you can skip some
	 * 		at certain intervals.
	 * @param numScans Number of scans requested, if numScans == 0 the data is
	 * 		supplied intil quit is called.
	 * @return returns if the command was succesfully sent
	 */
	bool scanRange(urg_range_data_byte_t comRange, 
		int startStep, int endStep, int clusterCount, int scanInterval,
		int numScans);


private:
	URG04LX(const URG04LX& rhs);
	URG04LX& operator = (const URG04LX& rhs);
	int _fd;
};
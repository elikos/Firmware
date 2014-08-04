/**
 * @file urg04lx.cpp
 * Hokuyo URG-04LX device driver.
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 

#include "ring_buffer.h"
#include "scip.h"

#include "urg04lx.h"

typedef enum {
	URG_DISTANCE,
	URG_DISTANCE_INTENSITY,
	URG_MULTIECHO,
	URG_MULTIECHO_INTENSITY,
	URG_STOP,
	URG_UNKNOWN,
} urg_measurement_type_t;

enum {
	URG_FALSE = 0,
	URG_TRUE = 1,

	BUFFER_SIZE = 64 + 2 + 6,

	EXPECTED_END = -1,

	RECEIVE_DATA_TIMEOUT,
	RECEIVE_DATA_COMPLETE,

	PP_RESPONSE_LINES = 10,
	VV_RESPONSE_LINES = 7,
	II_RESPONSE_LINES = 9,

	MAX_TIMEOUT = 140,
};

URG04LX::URG04LX(int fd)
{
	_fd = fd;
	char* buf = new char[256];
	ring_initialize(&_rbuf, buf, 6);
}

URG04LX::~URG04LX()
{
	delete _rbuf.buffer;
}

bool URG04LX::getVersion()
{
	write(_fd, "VV\n", 3);
	return true;
}

bool URG04LX::scanRange(urg_range_data_byte_t comRange, 
	int startStep, int endStep, int clusterCount, int scanInterval,
	int numScans)
{
	const char scanStartChar = 'M';
	const int SCAN_CMD_SIZE = 16;
	char buffer[SCAN_CMD_SIZE];
	char scanFormatChar;
	if(comRange == URG_COMMUNICATION_3_BYTE)
		scanFormatChar = 'D';
	else
		scanFormatChar = 'S';

	//min range = 10, max range = 750
	//add range padded with 0's
	int writeSize = snprintf(buffer, SCAN_CMD_SIZE, "%c%c%04d%04d%01d%02d\n", scanStartChar, 
		scanFormatChar, startStep, endStep, scanInterval, numScans);
	
	int writtenSize = write(_fd, buffer, SCAN_CMD_SIZE);
	if(writeSize != writtenSize)
		return false;
	else
		return true;
}
/*
bool verifyMeasurementResponse(urg_measurement_resp_t* response)
{
	//todo add more checks
	if(response->lf0 == '\n')
		return true;
	else
		return false;
}
*/
int* URG04LX::getRangeResponse(int* step, int* err)
{
	//see page 5 of http://www.hokuyo-aut.jp/02sensor/07scanner/download/pdf/URG_SCIP20.pdf
	if(!scanRange(URG_COMMUNICATION_3_BYTE, 44, 725, 99, 0, 1)){
		return 0;
	}

	const int EXPECTED_LF_COUNT = 6;
	int lfCount = 0;
	char buf = 8;
	ring_clear(&_rbuf);	//suboptimal... oh well
	while(lfCount < EXPECTED_LF_COUNT){
		if(read(_fd, &buf, 1) > 0){
			if(buf == '\n')
				lfCount += 1;
			ring_write(&_rbuf, &buf, 1);
		}
	}

	char* data = new char[ring_size(&_rbuf)];


	return 0;
}

bool URG04LX::recvByte()
{
	char c;
	if(::read(_fd, &c, 1) > 0){
		ring_write(&_rbuf, &c, 1);
		return true;
	} else {
		return false;
	}
}

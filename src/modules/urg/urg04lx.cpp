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
		int numScans, int _fd){
	const char scanStartChar = 'M';
	const int SCAN_CMD_SIZE = 22;
	char buffer[SCAN_CMD_SIZE];
	char scanFormatChar;
	//if(comRange == URG_COMMUNICATION_3_BYTE)
		scanFormatChar = 'D';
	//else
	//	scanFormatChar = 'S';

	//min range = 10, max range = 750
	//add range padded with 0's
	int writeSize = snprintf(buffer, SCAN_CMD_SIZE, "%c%c%04d%04d%02d%01d%02d;LOL\n", scanStartChar,
		scanFormatChar, startStep, endStep, clusterCount, scanInterval, numScans);
	
	int writtenSize = write(_fd, buffer, SCAN_CMD_SIZE);
	printf("Sent: %s", buffer);
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

int URG04LX::verifyStatus(uint8_t* statusArray){
	if(statusArray[0] == '0' && statusArray[1] == '0' && statusArray[2] == 'P')
		{return 0;}
	else if(statusArray[0] == '9' && statusArray[1] == '9' && statusArray[2] == 'b')
		{return 1;}
	else if(statusArray[0] == '0' && statusArray[1] != '0')
		{return 100;}
}

int URG04LX::byteDecode3(uint8_t* bytes)
{
	const uint8_t mask = 0x3f;	//keep first 6 bits
	bytes[0] -= 0x30;
	bytes[1] -= 0x30;
	bytes[2] -= 0x30;

	bytes[0] &= mask;
	bytes[1] &= mask;
	bytes[2] &= mask;

	int32_t result = (bytes[0] << 12) | (bytes[1] << 6) | bytes[2];
	return result;
}

int* URG04LX::getRangeResponse()
{
//	//see page 5 of http://www.hokuyo-aut.jp/02sensor/07scanner/download/pdf/URG_SCIP20.pdf
//	if(!scanRange(URG_COMMUNICATION_3_BYTE, 44, 725, 99, 0, 1)){
//		return 0;
//	}
//
//	const int EXPECTED_LF_COUNT = 6;
//	int lfCount = 0;
//	char buf = 8;
//	ring_clear(&_rbuf);	//suboptimal... oh well
//	while(lfCount < EXPECTED_LF_COUNT){
//		if(read(_fd, &buf, 1) > 0){
//			if(buf == '\n')
//				lfCount += 1;
//			ring_write(&_rbuf, &buf, 1);
//		}
//	}
//
//	char* data = new char[ring_size(&_rbuf)];
//
//
//	return 0;
	char buf = 0;
	int lfCount = 0;
	int distances[15];
	ring_buffer_t rbuf;
	char _rbuf[256];
	ring_initialize(&rbuf, _rbuf, 8);
	printf("\nscanning\n");
	const int _nbrScans = 1;
	const int _nbrCluster = 34;
	scanRange(URG_COMMUNICATION_3_BYTE, 130, 639, _nbrCluster , 0, _nbrScans, _fd);
	//scanRange(URG_COMMUNICATION_3_BYTE, 384, 384 , 1, 1, 1, fd);
	fflush(stdout);
	//read off all bytes until status bytes
	while(lfCount < 1){
		if(read(_fd, &buf, 1) > 0){
			printf("%c ", buf);
			fflush(stdout);
			if(buf == '\n')
				lfCount++;
		}
	}

	//read off status bytes
	int bytesRead = 0;
	uint8_t status[3];
	while(bytesRead < 3){
		if(read(_fd, &buf, 1) > 0){
			status[bytesRead++] = buf;
		}
	}
	fflush(stdout);

	if(verifyStatus(status) > 1){
		printf("bad status");
		//not the message 00P status
		int consecutiveLF = 0;
		while(consecutiveLF < 2){
			if(read(_fd, &buf, 1) > 0){
				if(buf == '\n') consecutiveLF++;
				else consecutiveLF = 0;
			}
		}
	}
	else {
		if(verifyStatus(status) == 0){
			//message is good (00P), pop off bytes until we get to the data
			lfCount = 0;
			while(lfCount < 5){
				if(read(_fd, &buf, 1) > 0){
					if(buf == '\n')
						lfCount++;
				}
			}
		} else if (verifyStatus(status) == 1){
			//message is good (99b), pop off bytes until we get to the data
			lfCount = 0;
			while(lfCount < 1){
				if(read(_fd, &buf, 2) > 0){
					if(buf == '\n')
						lfCount++;
				}
			}
		}

		uint8_t point[3];
		bytesRead = 0;
		bool keepGoing = true;
		int dataCount = 1;
		while(keepGoing){
			while(bytesRead < 3){
				if(read(_fd, &buf, 1) > 0){
						point[bytesRead] = buf;
						bytesRead++;
				}

			}

			if(point[0] != '\n' && point[1] != '\n' && point[2] != '\n'){
				distances[dataCount-1]= byteDecode3(point);
				printf("%d Decoded: %d\n", dataCount++, byteDecode3(point));
				fflush(stdout);
			}
			else{
				keepGoing = false;
				printf("End data\n");
				break;
			}
			bytesRead = 0;
		}
	}
	return distances;
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

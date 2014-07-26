/**
 * @file urg04lx.cpp
 * Hokuyo URG-04LX device driver.
 */

#include "urg04lx.h"

URG04LX::URG04LX(int fd)
{
	_fd = fd;
}
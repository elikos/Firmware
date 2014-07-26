/**
 * @file urg04lx.cpp
 * Hokuyo URG-04LX device driver.
 */

#include "URG04LX.h"

URG04LX::URG04LX(int fd)
{
	_fd = fd;
}
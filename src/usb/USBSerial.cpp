/*
 * Copyright (c) 2019 Gillou68310
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "USBSerial.h"

USBInterface* USBInterface::interfaces[TOTAL_INTERFACES] = {NULL};
int USBSerial::interface_count = 0;

USBSerial::USBSerial() {
	index = interface_count >> 1;
	data = new USBSerialDataInterface(interface_count + 1);
	com = new USBSerialComInterface(interface_count, data);
    interface_count += 2;
}

Result USBSerial::initialize(const char* str)
{
	Result rc = com->initialize(str);
	if(R_SUCCEEDED(rc)) rc = data->initialize(nullptr);
	return rc;
}

ssize_t USBSerial::read(char *ptr, size_t len)
{
	if(com->getDTE())
		return data->read(ptr, len, 1000000000LL/*U64_MAX*/);
	else
		return -1;
}

ssize_t USBSerial::write(const char *ptr, size_t len)
{
	if(com->getDTE())
		return data->write(ptr, len, 1000000000LL/*U64_MAX*/);
	else
		return -1;
}

ssize_t USBSerial::sendEvent(const char *ptr, size_t len)
{
	if(com->getDTE())
		return com->sendEvent(ptr, len, 1000000000LL/*U64_MAX*/);
	else
		return -1;
}

USBSerial::~USBSerial() {
	delete com;
	delete data;
}
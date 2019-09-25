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

#include "USBSerialDataInterface.h"
#include "USBSerialComInterface.h"

#define EP_IN 0
#define EP_OUT 1

USBSerialDataInterface::USBSerialDataInterface(int index) : USBInterface(index) {
}

USBSerialDataInterface::~USBSerialDataInterface() {
}

Result USBSerialDataInterface::initialize(const char* str)
{
    Result rc = usbAddStringDescriptor(&interface_descriptor.iInterface, "CDC ACM Data");
    if (R_SUCCEEDED(rc)) rc = usbInterfaceInit(getInterfaceIndex(), &interface_descriptor, NULL);
    if (R_SUCCEEDED(rc)) rc = usbAddEndpoint(getInterfaceIndex(), EP_IN, &endpoint_descriptor_in);
    if (R_SUCCEEDED(rc)) rc = usbAddEndpoint(getInterfaceIndex(), EP_OUT, &endpoint_descriptor_out);
    if (R_SUCCEEDED(rc)) rc = usbEnableInterface(getInterfaceIndex());
    return rc;
}

ssize_t USBSerialDataInterface::read(char *ptr, size_t len, u64 timestamp)
{
    return usbTransfer(getInterfaceIndex(), EP_OUT, UsbDirection_Read, (void*)ptr, len, timestamp);
}
ssize_t USBSerialDataInterface::write(const char *ptr, size_t len, u64 timestamp)
{
    return usbTransfer(getInterfaceIndex(), EP_IN, UsbDirection_Write, (void*)ptr, len, timestamp);
}